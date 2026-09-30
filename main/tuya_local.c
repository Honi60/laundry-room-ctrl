#include "tuya_local.h"

#include "config_manager.h"
#include "global_state.h"

#include "esp_check.h"
#include "esp_log.h"
#include "esp_rom_crc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "lwip/sockets.h"
#include "lwip/def.h"
#include "mbedtls/aes.h"

#include <string.h>
#include <stdlib.h>
#include <time.h>

#define TAG "tuya_local"

#define TUYA_PORT 6668
#define TUYA_HEADER 0x000055AA
#define TUYA_FOOTER 0x0000AA55

#define TUYA_CMD_CONTROL 0x07
#define TUYA_CMD_HEARTBEAT 0x09
#define TUYA_CMD_DP_QUERY  0x0A

#define TUYA_BUF_SIZE 512
#define TUYA_SOCK_TIMEOUT_MS 2000
#define TUYA_HEARTBEAT_MS    5000

typedef enum {
    TUYA_EVT_SET,
    TUYA_EVT_HEARTBEAT,
} tuya_event_type_t;

typedef struct {
    tuya_event_type_t type;
    bool on;
} tuya_event_t;

static QueueHandle_t s_queue = NULL;
static int s_fd = -1;
static uint32_t s_seq = 1;
static bool s_worker_running = false;

static int tuya_hex2byte(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

static bool tuya_get_key(const char *src, uint8_t key[16])
{
    size_t len = strnlen(src, CFG_TUYA_KEY_LEN);
    /* tolerate stray trailing whitespace/paste artifacts */
    while (len > 0 && (src[len - 1] == ' ' || src[len - 1] == '\t' ||
                       src[len - 1] == '\r' || src[len - 1] == '\n' ||
                       src[len - 1] == '\\')) {
        len--;
    }
    if (len == 0) return false;

    if (len == 16) {
        memcpy(key, src, 16);
        return true;
    }

    if (len == 32) {
        for (int i = 0; i < 16; i++) {
            int hi = tuya_hex2byte(src[i * 2]);
            int lo = tuya_hex2byte(src[i * 2 + 1]);
            if (hi < 0 || lo < 0) return false;
            key[i] = (uint8_t)((hi << 4) | lo);
        }
        return true;
    }

    return false;
}

static uint32_t tuya_next_seq(void)
{
    uint32_t seq = s_seq;
    s_seq++;
    if (s_seq == 0) s_seq = 1;
    return seq;
}

static void tuya_close(void)
{
    if (s_fd >= 0) {
        close(s_fd);
        s_fd = -1;
    }
    global_state_lock();
    global_state_get()->tuya_connected = false;
    global_state_unlock();
}

static int tuya_connect_locked(const app_config_t *cfg)
{
    if (cfg->tuya_ip[0] == '\0' || cfg->tuya_dev_id[0] == '\0') {
        return -1;
    }

    if (s_fd >= 0) return s_fd;

    int fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) {
        ESP_LOGE(TAG, "socket failed");
        return -1;
    }

    struct timeval tv = {
        .tv_sec = TUYA_SOCK_TIMEOUT_MS / 1000,
        .tv_usec = (TUYA_SOCK_TIMEOUT_MS % 1000) * 1000,
    };
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

    /* Strip an optional ":port" suffix - port is always TUYA_PORT */
    char ip[CFG_TUYA_IP_LEN];
    strlcpy(ip, cfg->tuya_ip, sizeof(ip));
    char *colon = strchr(ip, ':');
    if (colon) *colon = '\0';

    struct sockaddr_in addr = {0};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(TUYA_PORT);
    if (inet_aton(ip, &addr.sin_addr) == 0) {
        ESP_LOGE(TAG, "invalid IP '%s'", ip);
        close(fd);
        return -1;
    }

    if (connect(fd, (struct sockaddr *)&addr, sizeof(addr)) != 0) {
        ESP_LOGE(TAG, "connect to %s:%d failed", cfg->tuya_ip, TUYA_PORT);
        close(fd);
        return -1;
    }

    ESP_LOGI(TAG, "connected to %s:%d", cfg->tuya_ip, TUYA_PORT);

    global_state_lock();
    global_state_get()->tuya_connected = true;
    global_state_unlock();

    return fd;
}

static size_t tuya_build_packet(uint32_t seq, uint32_t cmd,
                                const uint8_t *payload, size_t payload_len,
                                const uint8_t key[16],
                                uint8_t *buf, size_t buf_len)
{
    if (cmd != TUYA_CMD_DP_QUERY && cmd != TUYA_CMD_CONTROL && cmd != TUYA_CMD_HEARTBEAT) {
        return 0;
    }

    size_t padded = ((payload_len + 15) / 16) * 16;
    if (padded == payload_len) padded += 16;
    uint8_t pad = (uint8_t)(padded - payload_len);

    bool with_prefix = (cmd != TUYA_CMD_DP_QUERY);
    size_t prefix_len = with_prefix ? 15 : 0;
    size_t total = 16 + prefix_len + padded + 8;
    if (total > buf_len) return 0;

    memset(buf, 0, total);

    uint32_t *p32 = (uint32_t *)buf;
    p32[0] = htonl(TUYA_HEADER);
    p32[1] = htonl(seq);
    p32[2] = htonl(cmd);
    p32[3] = htonl(prefix_len + padded + 8);

    if (with_prefix) {
        uint8_t *p = buf + 16;
        p[0] = '3';
        p[1] = '.';
        p[2] = '3';
    }

    uint8_t *p = buf + 16 + prefix_len;
    memcpy(p, payload, payload_len);
    for (size_t i = 0; i < pad; i++) {
        p[payload_len + i] = pad;
    }

    mbedtls_aes_context aes;
    mbedtls_aes_init(&aes);
    if (mbedtls_aes_setkey_enc(&aes, key, 128) != 0) {
        mbedtls_aes_free(&aes);
        return 0;
    }
    for (size_t i = 0; i < padded; i += 16) {
        mbedtls_aes_crypt_ecb(&aes, MBEDTLS_AES_ENCRYPT, p + i, p + i);
    }
    mbedtls_aes_free(&aes);

    uint32_t crc = esp_rom_crc32_le(0xFFFFFFFF, buf, 16 + prefix_len + padded);
    p32 = (uint32_t *)(buf + 16 + prefix_len + padded);
    p32[0] = htonl(crc);
    p32[1] = htonl(TUYA_FOOTER);

    return total;
}

static bool tuya_send_all(const uint8_t *buf, size_t len)
{
    if (s_fd < 0) return false;
    size_t sent = 0;
    while (sent < len) {
        int n = send(s_fd, buf + sent, len - sent, 0);
        if (n <= 0) {
            ESP_LOGE(TAG, "send failed");
            return false;
        }
        sent += n;
    }
    return true;
}

static bool tuya_recv_ack(void)
{
    uint8_t buf[TUYA_BUF_SIZE];
    int n = recv(s_fd, buf, sizeof(buf), 0);
    if (n <= 0) {
        ESP_LOGW(TAG, "recv ack failed: %d", n);
        return false;
    }
    ESP_LOGD(TAG, "recv %d bytes", n);
    return true;
}

static bool tuya_send_packet(uint32_t cmd, const uint8_t *payload, size_t payload_len,
                             const uint8_t key[16])
{
    if (s_fd < 0) return false;

    uint8_t buf[TUYA_BUF_SIZE];
    size_t len = tuya_build_packet(tuya_next_seq(), cmd, payload, payload_len, key, buf, sizeof(buf));
    if (len == 0) return false;

    if (!tuya_send_all(buf, len)) return false;

    if (!tuya_recv_ack()) return false;

    return true;
}

static bool tuya_ensure_connection(const app_config_t *cfg, const uint8_t key[16])
{
    if (s_fd < 0) {
        s_fd = tuya_connect_locked(cfg);
        if (s_fd < 0) return false;

        /* Some devices expect a DP query before accepting control. */
        char query[160];
        snprintf(query, sizeof(query),
                 "{\"devId\":\"%s\",\"gwId\":\"%s\",\"uid\":\"%s\",\"t\":\"%lu\"}",
                 cfg->tuya_dev_id, cfg->tuya_dev_id, cfg->tuya_dev_id,
                 (unsigned long)time(NULL));
        tuya_send_packet(TUYA_CMD_DP_QUERY, (const uint8_t *)query, strlen(query), key);
    }
    return true;
}

static void tuya_do_set(bool on, const app_config_t *cfg, const uint8_t key[16])
{
    if (!tuya_ensure_connection(cfg, key)) return;

    char dps[64];
    const char *v = on ? "true" : "false";
    if (cfg->tuya_dpid2) {
        /* Two-gang switch driving two lamps - set both channels together */
        snprintf(dps, sizeof(dps), "{\"%u\":%s,\"%u\":%s}",
                 cfg->tuya_dpid, v, cfg->tuya_dpid2, v);
    } else {
        snprintf(dps, sizeof(dps), "{\"%u\":%s}", cfg->tuya_dpid, v);
    }

    /* 3.3 devices may silently drop CONTROL without devId/uid/t fields */
    char payload[256];
    snprintf(payload, sizeof(payload),
             "{\"devId\":\"%s\",\"uid\":\"%s\",\"t\":\"%lu\",\"dps\":%s}",
             cfg->tuya_dev_id, cfg->tuya_dev_id,
             (unsigned long)time(NULL), dps);

    if (tuya_send_packet(TUYA_CMD_CONTROL, (const uint8_t *)payload, strlen(payload), key)) {
        ESP_LOGI(TAG, "switch set %s ok", on ? "on" : "off");
    } else {
        ESP_LOGE(TAG, "switch set %s failed", on ? "on" : "off");
        tuya_close();
    }
}

static void tuya_do_heartbeat(const app_config_t *cfg, const uint8_t key[16])
{
    if (s_fd < 0) {
        if (tuya_ensure_connection(cfg, key)) {
            ESP_LOGI(TAG, "reconnected");
        }
        return;
    }

    if (!tuya_send_packet(TUYA_CMD_HEARTBEAT, (const uint8_t *)"{}", 2, key)) {
        ESP_LOGW(TAG, "heartbeat failed, closing");
        tuya_close();
    }
}

static void tuya_worker_task(void *arg)
{
    (void)arg;
    s_worker_running = true;

    for (;;) {
        tuya_event_t evt;
        if (xQueueReceive(s_queue, &evt, pdMS_TO_TICKS(TUYA_HEARTBEAT_MS)) == pdPASS) {
            global_state_lock();
            app_config_t cfg = global_state_get()->config;
            global_state_unlock();

            uint8_t key[16];
            if (!tuya_get_key(cfg.tuya_local_key, key)) {
                if (evt.type == TUYA_EVT_SET) {
                    ESP_LOGW(TAG, "no Tuya local key configured");
                }
                continue;
            }

            if (cfg.tuya_ip[0] == '\0' || cfg.tuya_dev_id[0] == '\0') {
                if (evt.type == TUYA_EVT_SET) {
                    ESP_LOGW(TAG, "no Tuya IP/device configured");
                }
                continue;
            }

            if (evt.type == TUYA_EVT_SET) {
                tuya_do_set(evt.on, &cfg, key);
            }
        } else {
            /* Heartbeat timeout: keep the connection alive. */
            global_state_lock();
            app_config_t cfg = global_state_get()->config;
            global_state_unlock();

            uint8_t key[16];
            if (!tuya_get_key(cfg.tuya_local_key, key)) continue;
            if (cfg.tuya_ip[0] == '\0' || cfg.tuya_dev_id[0] == '\0') continue;

            tuya_do_heartbeat(&cfg, key);
        }
    }
}

esp_err_t tuya_local_init(void)
{
    s_queue = xQueueCreate(4, sizeof(tuya_event_t));
    if (!s_queue) return ESP_ERR_NO_MEM;

    xTaskCreatePinnedToCore(tuya_worker_task, "tuya_worker", 6144, NULL, 4, NULL, 0);
    ESP_LOGI(TAG, "Tuya local worker started");
    return ESP_OK;
}

esp_err_t tuya_local_set_switch(bool on)
{
    if (!s_queue) return ESP_FAIL;
    tuya_event_t evt = { .type = TUYA_EVT_SET, .on = on };
    if (xQueueSend(s_queue, &evt, pdMS_TO_TICKS(100)) != pdPASS) {
        ESP_LOGW(TAG, "queue full");
        return ESP_FAIL;
    }
    return ESP_OK;
}
