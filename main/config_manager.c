#include "config_manager.h"

#include "esp_check.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
#include <string.h>

#define TAG          "config"
#define NVS_NS       "appcfg"
#define CFG_KEY      "cfg_v1"

static nvs_handle_t s_nvs;

esp_err_t app_config_init(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_RETURN_ON_ERROR(ret, TAG, "nvs_flash_init failed");
    return nvs_open(NVS_NS, NVS_READWRITE, &s_nvs);
}

esp_err_t app_config_load_defaults(app_config_t *cfg) {
    if (cfg == NULL) return ESP_ERR_INVALID_ARG;
    memset(cfg, 0, sizeof(*cfg));

    cfg->cfg_version = CFG_VERSION;

    /* Safe factory defaults */
    cfg->light_thr_lux = 30.0f;
    cfg->temp_thr_c = 27.0f;
    cfg->hum_thr_pct = 65.0f;
    cfg->vacancy_timeout_ms = 5 * 60 * 1000; /* 5 min */

    cfg->auto_mode = true;
    cfg->fan_auto = true;
    cfg->light_auto = true;

    cfg->tuya_version = 33;
    cfg->tuya_dpid = 1;

    return ESP_OK;
}

esp_err_t app_config_load(app_config_t *cfg) {
    if (cfg == NULL) return ESP_ERR_INVALID_ARG;

    size_t len = sizeof(*cfg);
    esp_err_t ret = nvs_get_blob(s_nvs, CFG_KEY, cfg, &len);
    if (ret == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGW(TAG, "config not found, loading defaults and saving");
        ESP_RETURN_ON_ERROR(app_config_load_defaults(cfg), TAG, "defaults failed");
        return app_config_save(cfg);
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "nvs_get_blob failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Stored blob from an older firmware may be shorter than the current
       struct; zero the tail so appended fields get sane defaults. */
    if (len < sizeof(*cfg)) {
        memset((uint8_t *)cfg + len, 0, sizeof(*cfg) - len);
    }

    if (cfg->cfg_version != CFG_VERSION) {
        ESP_LOGW(TAG, "config version %lu != %d, resetting to defaults",
                 (unsigned long)cfg->cfg_version, CFG_VERSION);
        ESP_RETURN_ON_ERROR(app_config_load_defaults(cfg), TAG, "defaults failed");
        return app_config_save(cfg);
    }

    return ESP_OK;
}

esp_err_t app_config_save(const app_config_t *cfg) {
    if (cfg == NULL) return ESP_ERR_INVALID_ARG;
    esp_err_t ret = nvs_set_blob(s_nvs, CFG_KEY, cfg, sizeof(*cfg));
    if (ret != ESP_OK) return ret;
    return nvs_commit(s_nvs);
}
