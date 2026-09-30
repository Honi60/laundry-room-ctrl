#pragma once

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#define CFG_WIFI_SSID_LEN   32
#define CFG_WIFI_PASS_LEN   64
#define CFG_TUYA_IP_LEN     16
#define CFG_TUYA_DEV_ID_LEN 32
#define CFG_TUYA_KEY_LEN    32
#define CFG_VERSION         1

#define IR_MAX_SYMBOLS      128

typedef struct {
    /* Misc */
    uint32_t cfg_version;

    /* Wi-Fi */
    char     wifi_ssid[CFG_WIFI_SSID_LEN];
    char     wifi_pass[CFG_WIFI_PASS_LEN];

    /* Thresholds */
    float    light_thr_lux;
    float    temp_thr_c;
    float    hum_thr_pct;
    uint32_t vacancy_timeout_ms;

    /* Mode enables */
    bool     auto_mode;
    bool     fan_auto;
    bool     light_auto;

    /* Manual outputs */
    bool     fan_relay_manual;
    bool     fan_ir_manual;
    bool     light_manual;

    /* Tuya */
    char     tuya_ip[CFG_TUYA_IP_LEN];
    char     tuya_dev_id[CFG_TUYA_DEV_ID_LEN];
    char     tuya_local_key[CFG_TUYA_KEY_LEN];
    uint8_t  tuya_version;      /* 33, 34, 35 */
    uint8_t  tuya_dpid;

    /* IR raw symbols: each is a rmt_symbol_word_t serialized (duration0/level0/duration1/level1) */
    uint32_t ir_on_symbols[IR_MAX_SYMBOLS * 2];
    size_t   ir_on_count;
    uint32_t ir_off_symbols[IR_MAX_SYMBOLS * 2];
    size_t   ir_off_count;

    /* RF pulse sequences (future) */

    /* Second Tuya datapoint (e.g. 2-gang switch driving two lamps).
       0 = disabled. Appended at struct end so old NVS blobs stay valid. */
    uint8_t  tuya_dpid2;
} app_config_t;

esp_err_t app_config_init(void);
esp_err_t app_config_load(app_config_t *cfg);
esp_err_t app_config_save(const app_config_t *cfg);
esp_err_t app_config_load_defaults(app_config_t *cfg);
