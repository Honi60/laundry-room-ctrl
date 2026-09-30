#include "ld2410_hal.h"

#include <string.h>

#include "board.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "global_state.h"

#define TAG "ld2410_hal"

/* LD2410S minimal report frame (5 bytes):
   0x6E | state | dist_lo | dist_hi | 0x62
   state: 0/1 = no target, 2 = moving present, 3 = stationary present */
#define LD2410S_FRAME_HEAD 0x6E
#define LD2410S_FRAME_TAIL 0x62
#define LD2410S_FRAME_LEN  5

/* Command frame: FD FC FB FA | len(2 LE) | cmd(2 LE) | value | 04 03 02 01.
   ACK command word = cmd | 0x0100. While in config mode the sensor stops
   emitting report frames. */
#define LD2410_CMD_HEAD  0xFAFBFCFDu  /* little-endian: FD FC FB FA */
#define LD2410_CMD_TAIL  0x01020304u  /* little-endian: 04 03 02 01 */
#define LD2410_CMD_ENABLE   0x00FF
#define LD2410_CMD_END      0x00FE
#define LD2410_CMD_WR_PARAM 0x0070
#define LD2410_CMD_RD_PARAM 0x0071
#define LD2410_CMD_OUT_MODE 0x007A
#define LD2410_CMD_RD_VER   0x0000
#define LD2410_CMD_WR_SN    0x0010
#define LD2410_CMD_RD_SN    0x0011
#define LD2410_PARAM_UNMANNED_DELAY 0x0006
#define LD2410_PARAM_RESP_SPEED     0x000B

/* Sensor reports ~0.5Hz; tolerate a few missed frames before flagging absent.
   Presence itself decays faster so automation reacts to a stopped stream. */
#define LD2410_OK_TIMEOUT_MS      10000
#define LD2410_PRESENCE_TIMEOUT_MS 6000

/* Serialises command exchange against the streaming rx task. */
static SemaphoreHandle_t s_uart_mutex;
static volatile int s_dump_frames;   /* radardump: hex-dump N raw frames */

/* Full wire frame of the last command ACK (header..tail), for diagnostics. */
static uint8_t  s_last_frame[128];
static uint16_t s_last_frame_len;

void ld2410_dump_request(int count) { s_dump_frames = count; }

static void put_u16(uint8_t *p, uint16_t v) {
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)(v >> 8);
}

static void put_u32(uint8_t *p, uint32_t v) {
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)((v >> 8) & 0xFF);
    p[2] = (uint8_t)((v >> 16) & 0xFF);
    p[3] = (uint8_t)((v >> 24) & 0xFF);
}

/* Send a command frame and wait for its ACK. Report frames arriving in
   between are skipped. The sensor can take >1s to enter config mode,
   so the window is generous and enable-config is retried by the caller.
   Caller must hold s_uart_mutex. If resp is non-NULL the ACK's value bytes
   (data after the command-word echo, before the tail) are copied out. */
static esp_err_t ld2410_cmd(uint16_t cmd, const uint8_t *val, uint16_t vlen,
                            uint8_t *resp, uint16_t *resp_len) {
    uint8_t tx[32];
    if (vlen > sizeof(tx) - 12) return ESP_ERR_INVALID_SIZE;
    put_u32(tx, LD2410_CMD_HEAD);
    put_u16(tx + 4, (uint16_t)(vlen + 2));
    put_u16(tx + 6, cmd);
    if (vlen) memcpy(tx + 8, val, vlen);
    put_u32(tx + 8 + vlen, LD2410_CMD_TAIL);
    size_t txlen = 8 + vlen + 4;

    /* Scan for ACK: FD FC FB FA <len> <cmd|0x0100> <status...> 04 03 02 01.
       The sensor answers ~1.5ms after each command, and config mode must be
       exercised back-to-back — long gaps between commands let it drop out.
       So: send immediately, listen, resend every 600ms until deadline. */
    uint8_t rx[128];
    uint16_t want_cmd = (uint16_t)(cmd | 0x0100);
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(4000);
    TickType_t next_send = 0;
    size_t rlen = 0;
    uart_flush_input(LD2410_UART_PORT);
    while (xTaskGetTickCount() < deadline && rlen < sizeof(rx)) {
        if (xTaskGetTickCount() >= next_send) {
            uart_write_bytes(LD2410_UART_PORT, tx, txlen);
            next_send = xTaskGetTickCount() + pdMS_TO_TICKS(600);
        }
        int n = uart_read_bytes(LD2410_UART_PORT, rx + rlen,
                                sizeof(rx) - rlen, pdMS_TO_TICKS(50));
        if (n > 0) rlen += (size_t)n;
        for (size_t i = 0; i + 12 <= rlen; i++) {
            if (rx[i] != 0xFD || rx[i+1] != 0xFC || rx[i+2] != 0xFB ||
                rx[i+3] != 0xFA) continue;
            uint16_t alen = (uint16_t)rx[i+4] | ((uint16_t)rx[i+5] << 8);
            size_t flen = 4 + 2 + (size_t)alen + 4;
            if (i + flen > rlen) continue;   /* incomplete or false header */
            uint16_t acmd = (uint16_t)rx[i+6] | ((uint16_t)rx[i+7] << 8);
            ESP_LOGI(TAG, "cand i=%u acmd=%04x want=%04x tail=%02x%02x%02x%02x",
                     (unsigned)i, acmd, want_cmd,
                     rx[i+flen-4], rx[i+flen-3], rx[i+flen-2], rx[i+flen-1]);
            bool tail_ok = rx[i+flen-4] == 0x04 && rx[i+flen-3] == 0x03 &&
                           rx[i+flen-2] == 0x02 && rx[i+flen-1] == 0x01;
            if (acmd != want_cmd || !tail_ok) continue;
            s_last_frame_len = (flen <= sizeof(s_last_frame))
                               ? (uint16_t)flen : sizeof(s_last_frame);
            memcpy(s_last_frame, rx + i, s_last_frame_len);
            if (resp && resp_len) {
                uint16_t plen = (alen > 2) ? (uint16_t)(alen - 2) : 0;
                if (plen > *resp_len) plen = *resp_len;
                memcpy(resp, rx + i + 8, plen);
                *resp_len = plen;
            }
            return ESP_OK;
        }
    }
    if (rlen > 0) {
        ESP_LOG_BUFFER_HEX("ld2410_hal", rx, rlen);
    }
    return ESP_ERR_TIMEOUT;
}

esp_err_t ld2410_set_output_mode(bool standard) {
    if (!s_uart_mutex) return ESP_ERR_INVALID_STATE;
    esp_err_t ret = ESP_FAIL;
    xSemaphoreTake(s_uart_mutex, portMAX_DELAY);

    uint8_t v_en[2] = {0x01, 0x00};
    esp_err_t en = ESP_ERR_TIMEOUT;
    for (int retry = 0; retry < 3 && en != ESP_OK; retry++) {
        en = ld2410_cmd(LD2410_CMD_ENABLE, v_en, 2, NULL, NULL);
    }
    if (en != ESP_OK) {
        ret = ESP_ERR_TIMEOUT;
        goto done;
    }

    /* 0x007A: doc text says 00 00 00 01 00 00 = standard but the wire
       example shows 00 00 01 00 00 00 — the example is authoritative. */
    uint8_t v_mode[6] = {0x00, 0x00,
                         (uint8_t)(standard ? 0x01 : 0x00), 0x00, 0x00, 0x00};
    uint8_t resp[8];
    uint16_t rlen = sizeof(resp);
    ret = ld2410_cmd(LD2410_CMD_OUT_MODE, v_mode, 6, resp, &rlen);
    if (ret == ESP_OK && rlen >= 2 && (resp[0] != 0 || resp[1] != 0)) {
        ret = ESP_ERR_INVALID_RESPONSE;
    }
    if (ret == ESP_OK) {
        global_state_lock();
        global_state_get()->ld2410_std_mode = standard;
        global_state_unlock();
    }

    ld2410_cmd(LD2410_CMD_END, NULL, 0, NULL, NULL);

done:
    xSemaphoreGive(s_uart_mutex);
    return ret;
}

/* Read the 6 common params (0x0071). vals[] order:
   [0] far gate, [1] near gate, [2] unmanned delay (s),
   [3] status rate (x10 Hz), [4] distance rate (x10 Hz), [5] resp speed */
esp_err_t ld2410_read_params(uint32_t vals[6],
                             uint8_t *frame, uint16_t *frame_len) {
    if (!s_uart_mutex) return ESP_ERR_INVALID_STATE;
    static const uint16_t words[6] = {0x0005, 0x000A, 0x0006,
                                      0x0002, 0x000C, 0x000B};
    uint8_t v_req[12];
    for (int i = 0; i < 6; i++) put_u16(v_req + i * 2, words[i]);

    esp_err_t ret = ESP_FAIL;
    xSemaphoreTake(s_uart_mutex, portMAX_DELAY);

    uint8_t v_en[2] = {0x01, 0x00};
    esp_err_t en = ESP_ERR_TIMEOUT;
    for (int retry = 0; retry < 3 && en != ESP_OK; retry++) {
        en = ld2410_cmd(LD2410_CMD_ENABLE, v_en, 2, NULL, NULL);
    }
    if (en != ESP_OK) {
        ret = ESP_ERR_TIMEOUT;
        goto done;
    }

    uint8_t resp[64];
    uint16_t rlen = sizeof(resp);
    ret = ld2410_cmd(LD2410_CMD_RD_PARAM, v_req, sizeof(v_req), resp, &rlen);
    if (ret == ESP_OK) {
        /* grab the 0x0171 ACK frame before END overwrites s_last_frame */
        if (frame && frame_len) {
            uint16_t cp = (s_last_frame_len < *frame_len)
                          ? s_last_frame_len : *frame_len;
            memcpy(frame, s_last_frame, cp);
            *frame_len = cp;
        }
        /* resp = ACK status (2B) + 4-byte values in request order */
        if (rlen < 2 + 6 * 4) {
            ret = ESP_ERR_INVALID_SIZE;
        } else {
            for (int i = 0; i < 6; i++) {
                const uint8_t *p = resp + 2 + i * 4;
                vals[i] = (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
                          ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
            }
        }
    }
    ld2410_cmd(LD2410_CMD_END, NULL, 0, NULL, NULL);

done:
    xSemaphoreGive(s_uart_mutex);
    return ret;
}

esp_err_t ld2410_set_unmanned_delay(uint32_t seconds) {
    if (!s_uart_mutex) return ESP_ERR_INVALID_STATE;
    if (seconds < 10 || seconds > 120) return ESP_ERR_INVALID_ARG;

    esp_err_t ret = ESP_FAIL;
    xSemaphoreTake(s_uart_mutex, portMAX_DELAY);

    uint8_t v_en[2] = {0x01, 0x00};
    esp_err_t en = ESP_ERR_TIMEOUT;
    for (int retry = 0; retry < 3 && en != ESP_OK; retry++) {
        en = ld2410_cmd(LD2410_CMD_ENABLE, v_en, 2, NULL, NULL);
    }
    if (en != ESP_OK) {
        ret = ESP_ERR_TIMEOUT;
        goto done;
    }

    /* Write params: unmanned delay (word 0x0006) + fast response (0x000B=10) */
    uint8_t v_wr[12];
    put_u16(v_wr + 0, LD2410_PARAM_UNMANNED_DELAY);
    put_u32(v_wr + 2, seconds);
    put_u16(v_wr + 6, LD2410_PARAM_RESP_SPEED);
    put_u32(v_wr + 8, 10);   /* 10 = fast, 5 = normal */
    uint8_t wresp[8];
    uint16_t wlen = sizeof(wresp);
    ret = ld2410_cmd(LD2410_CMD_WR_PARAM, v_wr, 12, wresp, &wlen);
    if (ret == ESP_OK && wlen >= 2 && (wresp[0] != 0 || wresp[1] != 0)) {
        ret = ESP_ERR_INVALID_RESPONSE;
    }

    ld2410_cmd(LD2410_CMD_END, NULL, 0, NULL, NULL);

done:
    xSemaphoreGive(s_uart_mutex);
    return ret;
}

/* Raw exchange test: send one command frame, print TX hex, then dump every
   RX byte for `listen_ms`. Returns nothing — this is purely for eyeballing
   the wire exchange. Caller must hold s_uart_mutex. */
static void radarx_send(uint16_t cmd, const uint8_t *val, uint16_t vlen,
                        uint32_t listen_ms) {
    uint8_t tx[40];
    put_u32(tx, LD2410_CMD_HEAD);
    put_u16(tx + 4, (uint16_t)(vlen + 2));
    put_u16(tx + 6, cmd);
    if (vlen) memcpy(tx + 8, val, vlen);
    put_u32(tx + 8 + vlen, LD2410_CMD_TAIL);
    size_t txlen = 8 + vlen + 4;

    char hex[96];
    char *p = hex;
    for (size_t i = 0; i < txlen; i++) p += sprintf(p, "%02x ", tx[i]);
    ESP_LOGI(TAG, "tx: %s", hex);

    uart_write_bytes(LD2410_UART_PORT, tx, txlen);

    TickType_t end = xTaskGetTickCount() + pdMS_TO_TICKS(listen_ms);
    uint8_t rx[128];
    while (xTaskGetTickCount() < end) {
        int n = uart_read_bytes(LD2410_UART_PORT, rx, sizeof(rx),
                                pdMS_TO_TICKS(50));
        if (n <= 0) continue;
        p = hex;
        for (int i = 0; i < n; i++) p += sprintf(p, "%02x ", rx[i]);
        ESP_LOGI(TAG, "rx: %s", hex);
    }
}

/* Full wire-level transaction: enable cfg -> write SN -> read SN -> end cfg.
   Every byte in both directions is logged so the exchange can be verified
   against the protocol doc directly. */
void ld2410_exchange_test(void) {
    if (!s_uart_mutex) return;
    xSemaphoreTake(s_uart_mutex, portMAX_DELAY);
    uart_flush_input(LD2410_UART_PORT);

    /* enable config: cmd 0x00FF, value 01 00 */
    uint8_t en[2] = {0x01, 0x00};
    radarx_send(LD2410_CMD_ENABLE, en, 2, 1200);

    /* write SN: cmd 0x0010, value = len 08 00 + SN 11..88 */
    uint8_t wsn[10] = {0x08, 0x00, 0x11, 0x22, 0x33, 0x44,
                       0x55, 0x66, 0x77, 0x88};
    radarx_send(LD2410_CMD_WR_SN, wsn, 10, 1200);

    /* read SN: cmd 0x0011, no value */
    radarx_send(LD2410_CMD_RD_SN, NULL, 0, 1200);

    /* end config: cmd 0x00FE, no value */
    radarx_send(LD2410_CMD_END, NULL, 0, 1500);

    xSemaphoreGive(s_uart_mutex);
}

/* Send an arbitrary command inside one enable/end config session, with full
   TX/RX hex logging — lets any parameter variant be tried without reflashing. */
esp_err_t ld2410_raw_cmd(uint16_t cmd, const uint8_t *val, uint16_t vlen) {
    if (!s_uart_mutex) return ESP_ERR_INVALID_STATE;
    xSemaphoreTake(s_uart_mutex, portMAX_DELAY);
    uart_flush_input(LD2410_UART_PORT);

    uint8_t en[2] = {0x01, 0x00};
    radarx_send(LD2410_CMD_ENABLE, en, 2, 400);
    radarx_send(cmd, val, vlen, 1200);
    radarx_send(LD2410_CMD_END, NULL, 0, 800);

    xSemaphoreGive(s_uart_mutex);
    return ESP_OK;
}

esp_err_t ld2410_read_info(char *ver_buf, size_t ver_sz,
                           char *sn_buf, size_t sn_sz) {
    if (!s_uart_mutex) return ESP_ERR_INVALID_STATE;
    esp_err_t ret = ESP_FAIL;
    xSemaphoreTake(s_uart_mutex, portMAX_DELAY);

    uint8_t v_en[2] = {0x01, 0x00};
    esp_err_t en = ESP_ERR_TIMEOUT;
    for (int retry = 0; retry < 3 && en != ESP_OK; retry++) {
        en = ld2410_cmd(LD2410_CMD_ENABLE, v_en, 2, NULL, NULL);
    }
    if (en != ESP_OK) {
        ret = ESP_ERR_TIMEOUT;
        goto done;
    }

    /* Read firmware version: value = major(2) + minor(2) + patch(2) LE */
    uint8_t vresp[16];
    uint16_t vlen = sizeof(vresp);
    ret = ld2410_cmd(LD2410_CMD_RD_VER, NULL, 0, vresp, &vlen);
    if (ret == ESP_OK && ver_buf && ver_sz) {
        if (vlen >= 6) {
            uint16_t maj = (uint16_t)vresp[0] | ((uint16_t)vresp[1] << 8);
            uint16_t min = (uint16_t)vresp[2] | ((uint16_t)vresp[3] << 8);
            uint16_t pat = (uint16_t)vresp[4] | ((uint16_t)vresp[5] << 8);
            snprintf(ver_buf, ver_sz, "%u.%u.%u", maj, min, pat);
        } else {
            snprintf(ver_buf, ver_sz, "len=%u", vlen);
        }
    }

    /* Read serial number: value = status(2) + snlen(2) + sn(8 ASCII) */
    uint8_t sresp[16];
    uint16_t slen = sizeof(sresp);
    esp_err_t sret = ld2410_cmd(LD2410_CMD_RD_SN, NULL, 0, sresp, &slen);
    if (sret == ESP_OK && sn_buf && sn_sz) {
        if (slen >= 4 && sresp[0] == 0 && sresp[1] == 0) {
            uint16_t n = (uint16_t)sresp[2] | ((uint16_t)sresp[3] << 8);
            if (n > 8) n = 8;
            if (n > slen - 4) n = slen - 4;
            memcpy(sn_buf, sresp + 4, n);
            sn_buf[n] = '\0';
        } else {
            snprintf(sn_buf, sn_sz, "len=%u st=%02x%02x",
                     slen, slen > 1 ? sresp[1] : 0, slen > 0 ? sresp[0] : 0);
        }
    }

    ld2410_cmd(LD2410_CMD_END, NULL, 0, NULL, NULL);

done:
    xSemaphoreGive(s_uart_mutex);
    return ret;
}

static void ld2410s_handle_frame(uint8_t state, uint16_t dist,
                                 uint8_t energy, const uint32_t *gates,
                                 TickType_t *last_frame) {
    /* state: 0/1 = no target, 2/3 = target present (exact split unknown) */
    *last_frame = xTaskGetTickCount();
    global_state_lock();
    global_state_t *st = global_state_get();
    st->ld2410_ok = true;
    st->ld2410_state = state;
    if (gates) memcpy(st->ld2410_gate_energy, gates, 16 * sizeof(uint32_t));
    if (state >= 2) {
        st->presence = (state == 3) ? PRESENCE_STATIC : PRESENCE_MOVING;
        st->ld2410_moving_distance_cm = (uint8_t)(dist > 255 ? 255 : dist);
        st->ld2410_static_distance_cm = 0;
        st->ld2410_moving_energy = energy;
    } else {
        st->presence = PRESENCE_NONE;
        st->ld2410_moving_distance_cm = 0;
        st->ld2410_static_distance_cm = 0;
        st->ld2410_moving_energy = 0;
    }
    global_state_unlock();
}

/* Standard frame: F4 F3 F2 F1 | len(2) | type | ... | F8 F7 F6 F5.
   type 0x01 = target data (state, dist, 64B per-gate energy);
   type 0x03 = auto-threshold progress (2 bytes, progress*100). */
static void ld2410s_handle_std_frame(const uint8_t *f, size_t flen,
                                     TickType_t *last_frame) {
    if (flen < 8) return;
    switch (f[6]) {
    case 0x01: {
        /* head(4) len(2) type(1) state(1) dist(2) reserved(2) energy(64) tail */
        if (flen < 12 + 64 + 4) return;
        uint8_t state = f[7];
        uint16_t dist = (uint16_t)f[8] | ((uint16_t)f[9] << 8);
        uint32_t gates[16];
        uint32_t peak32 = 0;
        for (int g = 0; g < 16; g++) {
            const uint8_t *p = f + 12 + g * 4;
            gates[g] = (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
                       ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
            if (gates[g] > peak32) peak32 = gates[g];
        }
        uint8_t peak = (peak32 > 255) ? 255 : (uint8_t)peak32;
        ld2410s_handle_frame(state, dist, peak, gates, last_frame);
        break;
    }
    case 0x03: {
        uint16_t progress = (uint16_t)f[7] | ((uint16_t)f[8] << 8);
        ESP_LOGI(TAG, "auto-threshold progress: %.1f%%", progress / 100.0f);
        *last_frame = xTaskGetTickCount();
        break;
    }
    default:
        break;
    }
}

#define STD_HEAD0 0xF4
#define STD_HEAD_LEN 4
#define STD_TAIL_LEN 4

static void ld2410_task(void *arg) {
    (void)arg;
    uint8_t rxbuf[32];
    uint8_t frame[128];
    size_t  flen = 0;
    size_t  fwant = 0;      /* 0 = sync, else expected total frame length */
    bool    fstd = false;   /* standard-format frame in progress */
    uint32_t rx_total = 0;
    TickType_t last_frame = 0;

    for (;;) {
        /* Pause while a config command owns the UART. */
        xSemaphoreTake(s_uart_mutex, portMAX_DELAY);
        int n = uart_read_bytes(LD2410_UART_PORT, rxbuf, sizeof(rxbuf),
                                pdMS_TO_TICKS(20));
        xSemaphoreGive(s_uart_mutex);
        if (n > 0) {
            rx_total += (uint32_t)n;
            global_state_lock();
            global_state_get()->ld2410_rx_bytes = (uint16_t)rx_total;
            global_state_unlock();
        }
        for (int i = 0; i < n; i++) {
            uint8_t b = rxbuf[i];
            if (flen == 0) {
                if (b == LD2410S_FRAME_HEAD) {
                    frame[flen++] = b;
                    fwant = LD2410S_FRAME_LEN;
                    fstd = false;
                } else if (b == STD_HEAD0) {
                    frame[flen++] = b;
                    fwant = 0;
                    fstd = true;
                }
                continue;
            }
            /* resync on a new head byte */
            if (b == LD2410S_FRAME_HEAD || b == STD_HEAD0) {
                if (!(fstd && flen < STD_HEAD_LEN)) {
                    frame[0] = b;
                    flen = 1;
                    fstd = (b == STD_HEAD0);
                    fwant = fstd ? 0 : LD2410S_FRAME_LEN;
                    continue;
                }
            }
            frame[flen++] = b;
            if (fstd) {
                if (flen == STD_HEAD_LEN &&
                    (frame[1] != 0xF3 || frame[2] != 0xF2 || frame[3] != 0xF1)) {
                    flen = 0; fwant = 0; fstd = false;
                    continue;
                }
                if (flen == STD_HEAD_LEN + 2) {
                    uint16_t dlen = (uint16_t)frame[4] |
                                    ((uint16_t)frame[5] << 8);
                    if (dlen > sizeof(frame) - STD_HEAD_LEN - STD_TAIL_LEN - 2) {
                        flen = 0; fwant = 0; fstd = false;
                        continue;
                    }
                    fwant = STD_HEAD_LEN + 2 + dlen + STD_TAIL_LEN;
                }
            }
            if (fwant && flen == fwant) {
                if (s_dump_frames > 0) {
                    ESP_LOG_BUFFER_HEX(TAG, frame, flen);
                    s_dump_frames--;
                }
                if (fstd) {
                    if (frame[flen-4] == 0xF8 && frame[flen-3] == 0xF7 &&
                        frame[flen-2] == 0xF6 && frame[flen-1] == 0xF5) {
                        ld2410s_handle_std_frame(frame, flen, &last_frame);
                    }
                } else if (frame[4] == LD2410S_FRAME_TAIL) {
                    uint16_t dist = (uint16_t)frame[2] |
                                    ((uint16_t)frame[3] << 8);
                    ld2410s_handle_frame(frame[1], dist, 0, NULL, &last_frame);
                }
                flen = 0; fwant = 0; fstd = false;
            }
        }

        TickType_t now = xTaskGetTickCount();
        if (last_frame != 0) {
            if (now - last_frame > pdMS_TO_TICKS(LD2410_OK_TIMEOUT_MS)) {
                global_state_lock();
                global_state_t *st = global_state_get();
                st->ld2410_ok = false;
                if (now - last_frame > pdMS_TO_TICKS(LD2410_PRESENCE_TIMEOUT_MS)) {
                    st->presence = PRESENCE_NONE;
                }
                global_state_unlock();
            }
        }
    }
}

esp_err_t ld2410_hal_init(void) {
    s_uart_mutex = xSemaphoreCreateMutex();
    if (!s_uart_mutex) return ESP_ERR_NO_MEM;

    uart_config_t uart_config = {
        .baud_rate = LD2410_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_RETURN_ON_ERROR(uart_driver_install(LD2410_UART_PORT, 512, 0, 0, NULL, 0), TAG, "uart install");
    ESP_RETURN_ON_ERROR(uart_param_config(LD2410_UART_PORT, &uart_config), TAG, "uart config");
    ESP_RETURN_ON_ERROR(uart_set_pin(LD2410_UART_PORT, LD2410_TX_GPIO, LD2410_RX_GPIO,
                                     UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE), TAG, "uart pin");
    /* RX idles floating if the sensor's TX goes Hi-Z between frames;
       hold it high (UART idle level) so noise can't inject garbage. */
    gpio_set_pull_mode(LD2410_RX_GPIO, GPIO_PULLUP_ONLY);

    ESP_LOGI(TAG, "LD2410S on UART%d, TX=%d RX=%d, baud=%d (minimal report mode)",
             LD2410_UART_PORT, LD2410_TX_GPIO, LD2410_RX_GPIO, LD2410_BAUD_RATE);
    return ESP_OK;
}

void ld2410_hal_start_task(void) {
    xTaskCreatePinnedToCore(ld2410_task, "ld2410_task", 4096, NULL, 5, NULL, 1);
}
