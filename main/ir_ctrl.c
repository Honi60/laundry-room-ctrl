#include "ir_ctrl.h"

#include "board.h"
#include "config_manager.h"
#include "global_state.h"

#include "driver/gpio.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_rx.h"
#include "driver/rmt_encoder.h"
#include "driver/rmt_common.h"

#include "esp_check.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include <string.h>
#include <strings.h>
#include <stdlib.h>

#define TAG "ir_ctrl"

#define IR_RESOLUTION_HZ   1000000 /* 1 us per tick */
#define IR_CARRIER_HZ      38000
#define IR_CARRIER_DUTY    0.33

static uint32_t s_carrier_hz = IR_CARRIER_HZ;

#define IR_RX_BUF_SYMBOLS  (IR_MAX_SYMBOLS * 2)
#define IR_MIN_PULSE_US    100
#define IR_MAX_PULSE_US    30000
#define IR_LEARN_TIMEOUT_MS 10000

typedef struct {
    uint8_t  level;
    uint32_t duration;
} ir_edge_t;

static rmt_channel_handle_t s_tx_chan = NULL;
static rmt_channel_handle_t s_rx_chan = NULL;
static rmt_encoder_handle_t s_copy_encoder = NULL;
static QueueHandle_t        s_rx_queue = NULL;

static rmt_symbol_word_t    s_rx_buf[IR_RX_BUF_SYMBOLS];

static rmt_transmit_config_t s_tx_config = {
    .loop_count = 0,
    .flags = { .eot_level = 0 },
};

static rmt_receive_config_t s_rx_config = {
    .signal_range_min_ns = 100000,   /* 100 us, ignore glitches */
    .signal_range_max_ns = 30000000, /* 30 ms, end of frame */
};

static volatile bool s_learning = false;
static volatile bool s_learn_on = false;

static bool s_tx_ok = false;

static bool ir_rx_done_cb(rmt_channel_handle_t channel, const rmt_rx_done_event_data_t *edata, void *user_data)
{
    (void)channel;
    (void)user_data;
    BaseType_t high_task_wakeup = pdFALSE;
    if (s_learning) {
        xQueueSendFromISR(s_rx_queue, edata, &high_task_wakeup);
    }
    return high_task_wakeup == pdTRUE;
}

static size_t normalize_symbols(const rmt_symbol_word_t *in, size_t n,
                                rmt_symbol_word_t *out, size_t out_max)
{
    /* 4KB scratch - static so it doesn't blow the learn task's stack.
       Safe because only one learn task can run at a time (s_learning). */
    static ir_edge_t edges[IR_RX_BUF_SYMBOLS * 2];
    size_t ne = 0;

    for (size_t i = 0; i < n && ne < IR_RX_BUF_SYMBOLS * 2; i++) {
        /* Invert levels: the receiver is active-low, the LED is active-high. */
        if (in[i].duration0 > 0) {
            uint32_t d = in[i].duration0;
            if (d > 0x7FFF) d = 0x7FFF;
            edges[ne++] = (ir_edge_t){ .level = (uint8_t)(in[i].level0 ^ 1), .duration = d };
        }
        if (in[i].duration1 > 0) {
            uint32_t d = in[i].duration1;
            if (d > 0x7FFF) d = 0x7FFF;
            edges[ne++] = (ir_edge_t){ .level = (uint8_t)(in[i].level1 ^ 1), .duration = d };
        }
    }

    /* Discard leading idle (low) edges, start at the first mark (high). */
    size_t start = 0;
    while (start < ne && edges[start].level == 0) start++;
    if (start >= ne) return 0;

    /* Cap the final trailing idle so replays don't wait too long. */
    if (edges[ne - 1].level == 0 && edges[ne - 1].duration > 10000) {
        edges[ne - 1].duration = 10000;
    }

    size_t out_n = 0;
    for (size_t i = start; i < ne && out_n < out_max; i += 2) {
        rmt_symbol_word_t sym = {0};
        sym.level0 = edges[i].level;
        sym.duration0 = edges[i].duration;
        if (i + 1 < ne) {
            sym.level1 = edges[i + 1].level;
            sym.duration1 = edges[i + 1].duration;
        } else {
            /* Odd number of edges: add a short trailing space. */
            sym.level1 = 0;
            sym.duration1 = 1000;
        }
        out[out_n++] = sym;
    }
    return out_n;
}

static void ir_learn_task(void *arg)
{
    bool learn_on = (bool)(uintptr_t)arg;
    s_learning = true;
    s_learn_on = learn_on;

    rmt_rx_done_event_data_t rx_data;
    size_t symbol_count = 0;
    bool ok = false;

    if (rmt_enable(s_rx_chan) != ESP_OK) {
        ESP_LOGE(TAG, "learn: failed to enable RX");
        goto done;
    }

    if (rmt_receive(s_rx_chan, s_rx_buf, sizeof(s_rx_buf), &s_rx_config) != ESP_OK) {
        ESP_LOGE(TAG, "learn: failed to start receive");
        rmt_disable(s_rx_chan);
        goto done;
    }

    ESP_LOGI(TAG, "learn %s: waiting for IR...", learn_on ? "on" : "off");
    if (xQueueReceive(s_rx_queue, &rx_data, pdMS_TO_TICKS(IR_LEARN_TIMEOUT_MS)) == pdPASS) {
        rmt_symbol_word_t raw[IR_RX_BUF_SYMBOLS];
        size_t n = rx_data.num_symbols;
        if (n > IR_RX_BUF_SYMBOLS) n = IR_RX_BUF_SYMBOLS;
        memcpy(raw, rx_data.received_symbols, n * sizeof(rmt_symbol_word_t));

        rmt_disable(s_rx_chan);

        rmt_symbol_word_t normalized[IR_RX_BUF_SYMBOLS];
        symbol_count = normalize_symbols(raw, n, normalized, IR_RX_BUF_SYMBOLS);
        if (symbol_count == 0 || symbol_count > IR_MAX_SYMBOLS * 2) {
            ESP_LOGW(TAG, "learn %s: no usable IR frame", learn_on ? "on" : "off");
        } else {
            global_state_lock();
            app_config_t *cfg = &global_state_get()->config;
            uint32_t *dst = learn_on ? cfg->ir_on_symbols : cfg->ir_off_symbols;
            size_t *cnt = learn_on ? &cfg->ir_on_count : &cfg->ir_off_count;
            for (size_t i = 0; i < symbol_count; i++) {
                dst[i] = normalized[i].val;
            }
            *cnt = symbol_count;
            esp_err_t ret = app_config_save(cfg);
            global_state_unlock();
            ok = (ret == ESP_OK);
            ESP_LOGI(TAG, "learn %s: stored %u symbols (save %s)",
                     learn_on ? "on" : "off",
                     (unsigned)symbol_count,
                     ok ? "ok" : "failed");
        }
    } else {
        rmt_disable(s_rx_chan);
        ESP_LOGW(TAG, "learn %s: timeout", learn_on ? "on" : "off");
    }

done:
    s_learning = false;
    s_learn_on = false;
    vTaskDelete(NULL);
}

esp_err_t ir_ctrl_init(void)
{
    rmt_tx_channel_config_t tx_cfg = {
        .gpio_num = IR_TX_GPIO,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = IR_RESOLUTION_HZ,
        .mem_block_symbols = 64,
        .trans_queue_depth = 2,
    };
    ESP_RETURN_ON_ERROR(rmt_new_tx_channel(&tx_cfg, &s_tx_chan), TAG, "new tx channel");

    rmt_rx_channel_config_t rx_cfg = {
        .gpio_num = IR_RX_GPIO,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = IR_RESOLUTION_HZ,
        .mem_block_symbols = IR_RX_BUF_SYMBOLS,
    };
    ESP_RETURN_ON_ERROR(rmt_new_rx_channel(&rx_cfg, &s_rx_chan), TAG, "new rx channel");

    /* Carrier stays OFF at idle: on this ESP32 the RMT carrier keeps
       free-running once enabled, so it is only applied around bursts. */

    rmt_copy_encoder_config_t copy_cfg = {};
    ESP_RETURN_ON_ERROR(rmt_new_copy_encoder(&copy_cfg, &s_copy_encoder), TAG, "new copy encoder");

    s_rx_queue = xQueueCreate(2, sizeof(rmt_rx_done_event_data_t));
    if (!s_rx_queue) return ESP_ERR_NO_MEM;

    rmt_rx_event_callbacks_t cbs = {
        .on_recv_done = ir_rx_done_cb,
    };
    ESP_RETURN_ON_ERROR(rmt_rx_register_event_callbacks(s_rx_chan, &cbs, NULL), TAG, "rx callback");

    ESP_RETURN_ON_ERROR(rmt_enable(s_tx_chan), TAG, "enable tx");

    /* Weakest pad drive (~5mA): protects the pin if the IR LED is wired
       directly without a series resistor (point-blank mounting). */
    gpio_set_drive_capability(IR_TX_GPIO, GPIO_DRIVE_CAP_0);

    s_tx_ok = true;
    ESP_LOGI(TAG, "IR RMT ready, TX=%d RX=%d", IR_TX_GPIO, IR_RX_GPIO);
    return ESP_OK;
}

esp_err_t ir_ctrl_set_carrier(uint32_t freq_hz)
{
    if (!s_tx_chan) return ESP_ERR_INVALID_STATE;
    if (freq_hz) s_carrier_hz = freq_hz;
    rmt_carrier_config_t carrier_cfg = {
        .frequency_hz = freq_hz,
        .duty_cycle = IR_CARRIER_DUTY,
    };
    return rmt_apply_carrier(s_tx_chan, &carrier_cfg);
}

static void ir_carrier_on(void)  { ir_ctrl_set_carrier(s_carrier_hz); }
static void ir_carrier_off(void) { ir_ctrl_set_carrier(0); }

/* Fan remote codes (duty-cycle, ~1.7ms cell): 12 bits per frame. The 12th
   bit's mark doubles as the end-of-frame trailer; its gap runs into the
   inter-frame space. Empty string falls back to the learned code. */
#define IR_FAN_ON_PATTERN    "SSLSSLLLLLSL"
#define IR_FAN_OFF_PATTERN   "SSLSSLLLLLLS"
#define IR_FAN_TIMER_PATTERN "SSLSSLLLSLLL"
#define IR_FAN_MODE_PATTERN  "SSLSSLLLLSLL"
#define IR_FAN_SWING_PATTERN "SSLSSLLSLLLL"

void ir_ctrl_send_pattern(const char *pattern, const uint32_t t_us[4],
                          int repeats);

void ir_ctrl_send_fan(const char *cmd)
{
    if (!s_tx_ok || !cmd) return;
    if (!strcasecmp(cmd, "on"))        { ir_ctrl_send(true);  return; }
    if (!strcasecmp(cmd, "off"))       { ir_ctrl_send(false); return; }
    if (!strcasecmp(cmd, "timer"))     { ir_ctrl_send_pattern(IR_FAN_TIMER_PATTERN, NULL, 0); return; }
    if (!strcasecmp(cmd, "mode"))      { ir_ctrl_send_pattern(IR_FAN_MODE_PATTERN,  NULL, 0); return; }
    if (!strcasecmp(cmd, "swing"))     { ir_ctrl_send_pattern(IR_FAN_SWING_PATTERN, NULL, 0); return; }
    ESP_LOGW(TAG, "irfan: unknown command '%s'", cmd);
}

void ir_ctrl_send(bool on)
{
    if (!s_tx_ok) return;

    const char *pattern = on ? IR_FAN_ON_PATTERN : IR_FAN_OFF_PATTERN;
    if (pattern[0]) {
        ir_ctrl_send_pattern(pattern, NULL, 0);
        return;
    }

    rmt_symbol_word_t tx_symbols[IR_MAX_SYMBOLS * 2];
    size_t count = 0;

    global_state_lock();
    app_config_t *cfg = &global_state_get()->config;
    uint32_t *src = on ? cfg->ir_on_symbols : cfg->ir_off_symbols;
    count = on ? cfg->ir_on_count : cfg->ir_off_count;
    if (count > IR_MAX_SYMBOLS * 2) count = IR_MAX_SYMBOLS * 2;
    for (size_t i = 0; i < count; i++) {
        tx_symbols[i].val = src[i];
    }
    global_state_unlock();

    if (count == 0) {
        ESP_LOGW(TAG, "send %s: no learned code", on ? "on" : "off");
        return;
    }
    /* Sacrificial tail items: the ESP32 RMT clips the end of the stream,
       so real symbols must not sit at the tail. */
    for (int i = 0; i < 3 && count < sizeof(tx_symbols) / sizeof(tx_symbols[0]); i++) {
        tx_symbols[count++] = (rmt_symbol_word_t){
            .level0 = 0, .duration0 = 700, .level1 = 0, .duration1 = 700 };
    }

    ESP_LOGI(TAG, "sending IR %s (%u symbols)", on ? "on" : "off", (unsigned)count);
    ir_carrier_on();
    if (rmt_transmit(s_tx_chan, s_copy_encoder, tx_symbols,
                     count * sizeof(rmt_symbol_word_t), &s_tx_config) != ESP_OK) {
        ir_carrier_off();
        ESP_LOGE(TAG, "send %s: rmt_transmit failed", on ? "on" : "off");
        return;
    }
    rmt_tx_wait_all_done(s_tx_chan, 1000);
    esp_rom_delay_us(1000); /* settle before killing the carrier */
    ir_carrier_off();
}

/* Fan-remote duty-cycle code: constant ~1.7ms bit cell.
   'S' = long mark + short gap, 'L' = short mark + long gap.
   t_us = {s_mark, s_gap, l_mark, l_gap} measured on the scope. */
#define IR_S_MARK_US 1240
#define IR_S_GAP_US  440
#define IR_L_MARK_US 400
#define IR_L_GAP_US  1240
/* RC repeats the frame with ~8.4ms between bursts (measured). Our silence
   after the last mark = last bit's gap (0.44/1.24ms) + sacrificial tail
   (4.2ms) + settle (1ms) + this delay. 2000us puts the total at ~8.4ms. */
#define IR_FRAME_SPACE_US 2000
#define IR_SEND_REPEATS     5

void ir_ctrl_send_pattern(const char *pattern, const uint32_t t_us[4],
                          int repeats)
{
    if (!s_tx_ok || !pattern) return;
    uint32_t t[4] = {IR_S_MARK_US, IR_S_GAP_US, IR_L_MARK_US, IR_L_GAP_US};
    if (t_us) memcpy(t, t_us, sizeof(t));
    if (repeats < 1) repeats = IR_SEND_REPEATS;

    rmt_symbol_word_t syms[48];
    size_t n = 0;
    for (const char *p = pattern; *p && n < sizeof(syms) / sizeof(syms[0]); p++) {
        uint32_t mark, gap;
        if (*p == 'S' || *p == 's')      { mark = t[0]; gap = t[1]; }
        else if (*p == 'L' || *p == 'l') { mark = t[2]; gap = t[3]; }
        else continue;
        if (mark > 0x7FFF) mark = 0x7FFF;
        if (gap  > 0x7FFF) gap  = 0x7FFF;
        syms[n].level0 = 1; syms[n].duration0 = mark;
        syms[n].level1 = 0; syms[n].duration1 = gap;
        n++;
    }
    if (n == 0) return;
    /* Sacrificial tail: on this ESP32 the RMT clips the last item(s) of
       the stream, so the final data bit must NOT be last. Three dummy
       low items absorb whatever the hardware eats - pure silence. */
    for (int i = 0; i < 3 && n < sizeof(syms) / sizeof(syms[0]); i++) {
        syms[n++] = (rmt_symbol_word_t){
            .level0 = 0, .duration0 = 700, .level1 = 0, .duration1 = 700 };
    }

    ESP_LOGI(TAG, "irsend pattern '%s' x%d (%u syms)", pattern, repeats, (unsigned)n);
    for (int r = 0; r < repeats; r++) {
        ir_carrier_on();
        if (rmt_transmit(s_tx_chan, s_copy_encoder, syms,
                         n * sizeof(rmt_symbol_word_t), &s_tx_config) != ESP_OK) {
            ir_carrier_off();
            ESP_LOGE(TAG, "irsend: rmt_transmit failed");
            return;
        }
        rmt_tx_wait_all_done(s_tx_chan, 1000);
        esp_rom_delay_us(1000); /* settle before killing the carrier */
        ir_carrier_off();
        /* remotes re-send the frame every ~8.4ms while the button is held */
        if (r + 1 < repeats) esp_rom_delay_us(IR_FRAME_SPACE_US);
    }
}

/* Loopback test: aim the TX LED at the VS1838B on IR_RX_GPIO, run
   'irloop SSLSSLLLLLS' — prints every mark/gap the receiver decoded,
   i.e. exactly what the fan's own receiver would see. */
void ir_ctrl_loopback(const char *pattern)
{
    if (!s_tx_ok || !pattern || !pattern[0]) return;
    if (s_learning) {
        ESP_LOGW(TAG, "irloop: busy");
        return;
    }
    s_learning = true;
    if (rmt_enable(s_rx_chan) != ESP_OK ||
        rmt_receive(s_rx_chan, s_rx_buf, sizeof(s_rx_buf), &s_rx_config) != ESP_OK) {
        ESP_LOGE(TAG, "irloop: failed to arm RX");
        rmt_disable(s_rx_chan);
        s_learning = false;
        return;
    }

    ir_ctrl_send_pattern(pattern, NULL, 1);

    rmt_rx_done_event_data_t rx_data;
    if (xQueueReceive(s_rx_queue, &rx_data, pdMS_TO_TICKS(600)) == pdPASS) {
        size_t n = rx_data.num_symbols;
        ESP_LOGI(TAG, "irloop '%s': %u raw symbols", pattern, (unsigned)n);
        char line[96];
        for (size_t i = 0; i < n && i < 40; i++) {
            rmt_symbol_word_t w = rx_data.received_symbols[i];
            /* VS1838B is active-low: pin low during a mark -> print level^1 */
            snprintf(line, sizeof(line), "  %c%-5u %c%-5u",
                     (w.level0 ^ 1) ? 'M' : 'g', (unsigned)w.duration0,
                     (w.level1 ^ 1) ? 'M' : 'g', (unsigned)w.duration1);
            ESP_LOGI(TAG, "%s", line);
        }
    } else {
        ESP_LOGW(TAG, "irloop: nothing received");
    }
    rmt_disable(s_rx_chan);
    s_learning = false;
}

/* Software carrier: bake the ~38kHz square wave into the symbol stream
   (9us high + 17us low = ~38.5kHz, ~35% duty). Bypasses the RMT carrier
   generator entirely - every mark is plain symbol data the hardware
   cannot drop or leave unmodulated. */
#define IR_PWM_HI_US   9
#define IR_PWM_LO_US   17
#define IR_PWM_PERIOD  (IR_PWM_HI_US + IR_PWM_LO_US)

void ir_ctrl_send_pattern_pwm(const char *pattern, const uint32_t t_us[4],
                              int repeats)
{
    if (!s_tx_ok || !pattern) return;
    uint32_t t[4] = {IR_S_MARK_US, IR_S_GAP_US, IR_L_MARK_US, IR_L_GAP_US};
    if (t_us) memcpy(t, t_us, sizeof(t));
    if (repeats < 1) repeats = IR_SEND_REPEATS;

    static rmt_symbol_word_t syms[1536]; /* 6KB, single-shot use */
    const size_t cap = sizeof(syms) / sizeof(syms[0]);
    size_t n = 0;
    for (const char *p = pattern; *p && n + 128 < cap; p++) {
        uint32_t mark, gap;
        if (*p == 'S' || *p == 's')      { mark = t[0]; gap = t[1]; }
        else if (*p == 'L' || *p == 'l') { mark = t[2]; gap = t[3]; }
        else continue;
        size_t cyc = mark / IR_PWM_PERIOD;
        gap += mark - cyc * IR_PWM_PERIOD;  /* fold mark remainder into gap */
        if (gap > 0x7FFF) gap = 0x7FFF;
        for (size_t c = 0; c < cyc && n < cap; c++) {
            syms[n++] = (rmt_symbol_word_t){
                .level0 = 1, .duration0 = IR_PWM_HI_US,
                .level1 = 0, .duration1 = IR_PWM_LO_US };
        }
        syms[n++] = (rmt_symbol_word_t){
            .level0 = 0, .duration0 = gap, .level1 = 0, .duration1 = 1 };
    }
    if (n == 0) return;
    /* sacrificial tail - the RMT clips stream-end items */
    for (int i = 0; i < 3 && n < cap; i++) {
        syms[n++] = (rmt_symbol_word_t){
            .level0 = 0, .duration0 = 700, .level1 = 0, .duration1 = 700 };
    }

    ESP_LOGI(TAG, "irpwm '%s' x%d (%u syms)", pattern, repeats, (unsigned)n);
    ir_carrier_off(); /* carrier must stay off - the stream IS the carrier */
    for (int r = 0; r < repeats; r++) {
        if (rmt_transmit(s_tx_chan, s_copy_encoder, syms,
                         n * sizeof(rmt_symbol_word_t), &s_tx_config) != ESP_OK) {
            ESP_LOGE(TAG, "irpwm: rmt_transmit failed");
            return;
        }
        rmt_tx_wait_all_done(s_tx_chan, 2000);
        if (r + 1 < repeats) esp_rom_delay_us(IR_FRAME_SPACE_US);
    }
}

/* Passive capture: listen on the VS1838B for ~8s and print every
   mark/gap edge. Point any remote at it to see its real waveform. */
void ir_ctrl_capture(void)
{
    if (s_learning) {
        ESP_LOGW(TAG, "ircap: busy");
        return;
    }
    s_learning = true;
    if (rmt_enable(s_rx_chan) != ESP_OK ||
        rmt_receive(s_rx_chan, s_rx_buf, sizeof(s_rx_buf), &s_rx_config) != ESP_OK) {
        ESP_LOGE(TAG, "ircap: failed to arm RX");
        rmt_disable(s_rx_chan);
        s_learning = false;
        return;
    }
    ESP_LOGI(TAG, "ircap: listening 8s, press a remote button...");
    rmt_rx_done_event_data_t rx_data;
    if (xQueueReceive(s_rx_queue, &rx_data, pdMS_TO_TICKS(8000)) == pdPASS) {
        size_t n = rx_data.num_symbols;
        ESP_LOGI(TAG, "ircap: %u raw symbols", (unsigned)n);
        for (size_t i = 0; i < n && i < 64; i++) {
            rmt_symbol_word_t w = rx_data.received_symbols[i];
            ESP_LOGI(TAG, "  %c%-5u %c%-5u",
                     (w.level0 ^ 1) ? 'M' : 'g', (unsigned)w.duration0,
                     (w.level1 ^ 1) ? 'M' : 'g', (unsigned)w.duration1);
        }
    } else {
        ESP_LOGW(TAG, "ircap: nothing received");
    }
    rmt_disable(s_rx_chan);
    s_learning = false;
}

void ir_ctrl_learn(bool on)
{
    if (s_learning) {
        ESP_LOGW(TAG, "learn already in progress");
        return;
    }
    xTaskCreate(ir_learn_task, "ir_learn", 6144, (void *)(uintptr_t)on, 5, NULL);
}

bool ir_ctrl_is_learning(void)
{
    return s_learning;
}
