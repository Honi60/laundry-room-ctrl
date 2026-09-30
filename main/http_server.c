#include "http_server.h"

#include "bt_spp.h"
#include "config_manager.h"
#include "global_state.h"
#include "cli.h"
#include "esp_log.h"
#include "esp_http_server.h"
#include "mdns.h"
#include "cJSON.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#define TAG "http_server"

extern const uint8_t index_html_start[] asm("_binary_index_html_start");
extern const uint8_t index_html_end[]   asm("_binary_index_html_end");
extern const uint8_t laundary_room_svg_start[] asm("_binary_laundary_room_svg_start");
extern const uint8_t laundary_room_svg_end[]   asm("_binary_laundary_room_svg_end");
extern const uint8_t manifest_json_start[] asm("_binary_manifest_json_start");
extern const uint8_t manifest_json_end[]   asm("_binary_manifest_json_end");
extern const uint8_t icon_192_png_start[] asm("_binary_icon_192_png_start");
extern const uint8_t icon_192_png_end[]   asm("_binary_icon_192_png_end");
extern const uint8_t icon_512_png_start[] asm("_binary_icon_512_png_start");
extern const uint8_t icon_512_png_end[]   asm("_binary_icon_512_png_end");
extern const uint8_t apple_touch_icon_png_start[] asm("_binary_apple_touch_icon_png_start");
extern const uint8_t apple_touch_icon_png_end[]   asm("_binary_apple_touch_icon_png_end");

static httpd_handle_t s_server = NULL;
static bool s_mdns_started = false;

static esp_err_t send_embedded(httpd_req_t *req, const uint8_t *start,
                               const uint8_t *end, const char *type) {
    httpd_resp_set_type(req, type);
    httpd_resp_send(req, (const char *)start, end - start);
    return ESP_OK;
}

static esp_err_t root_get_handler(httpd_req_t *req) {
    return send_embedded(req, index_html_start, index_html_end, "text/html");
}

static esp_err_t icon_svg_get_handler(httpd_req_t *req) {
    return send_embedded(req, laundary_room_svg_start, laundary_room_svg_end, "image/svg+xml");
}

static esp_err_t manifest_get_handler(httpd_req_t *req) {
    return send_embedded(req, manifest_json_start, manifest_json_end, "application/manifest+json");
}

static esp_err_t icon_192_get_handler(httpd_req_t *req) {
    return send_embedded(req, icon_192_png_start, icon_192_png_end, "image/png");
}

static esp_err_t icon_512_get_handler(httpd_req_t *req) {
    return send_embedded(req, icon_512_png_start, icon_512_png_end, "image/png");
}

static esp_err_t apple_icon_get_handler(httpd_req_t *req) {
    return send_embedded(req, apple_touch_icon_png_start, apple_touch_icon_png_end, "image/png");
}

static esp_err_t favicon_get_handler(httpd_req_t *req) {
    return send_embedded(req, icon_192_png_start, icon_192_png_end, "image/png");
}

static esp_err_t status_get_handler(httpd_req_t *req) {
    /* Small snapshot only - a full global_state_t is ~2.3KB, too big for a task stack */
    status_snapshot_t st;
    global_state_status_snapshot(&st);

    cJSON *root = cJSON_CreateObject();
    cJSON_AddBoolToObject(root, "auto", st.auto_mode);
    cJSON_AddBoolToObject(root, "fan_auto", st.fan_auto);
    cJSON_AddBoolToObject(root, "light_auto", st.light_auto);
    cJSON_AddBoolToObject(root, "relay", st.fan_relay);
    cJSON_AddBoolToObject(root, "ir_fan", st.fan_ir);
    cJSON_AddBoolToObject(root, "light", st.light);
    cJSON_AddBoolToObject(root, "wifi_connected", st.wifi_connected);
    cJSON_AddNumberToObject(root, "rssi", st.wifi_rssi);
    cJSON_AddStringToObject(root, "ip", st.ip_addr);
    cJSON_AddStringToObject(root, "hostname", "laundry_room.local");
    cJSON_AddStringToObject(root, "bt_name", BT_DEVICE_NAME);
    cJSON_AddBoolToObject(root, "aht_ok", st.aht_ok);
    cJSON_AddBoolToObject(root, "bmp_ok", st.bmp_ok);
    cJSON_AddBoolToObject(root, "veml_ok", st.veml_ok);
    cJSON_AddNumberToObject(root, "lux", st.lux);
    cJSON_AddNumberToObject(root, "temperature", st.temperature);
    cJSON_AddNumberToObject(root, "humidity", st.humidity);
    cJSON_AddNumberToObject(root, "pressure", st.pressure);
    cJSON_AddBoolToObject(root, "ld2410_ok", st.ld2410_ok);
    cJSON_AddBoolToObject(root, "ld2410_std_mode", st.ld2410_std_mode);
    cJSON_AddNumberToObject(root, "presence", st.presence);
    cJSON_AddNumberToObject(root, "moving_dist", st.moving_distance_cm);
    cJSON_AddNumberToObject(root, "moving_energy", st.moving_energy);
    cJSON_AddNumberToObject(root, "static_dist", st.static_distance_cm);
    cJSON_AddNumberToObject(root, "static_energy", st.static_energy);

    char *s = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, s, strlen(s));
    free(s);
    return ESP_OK;
}

static esp_err_t control_post_handler(httpd_req_t *req) {
    char buf[128];
    int ret = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (ret <= 0) return ESP_FAIL;
    buf[ret] = '\0';

    cJSON *root = cJSON_Parse(buf);
    if (!root) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad json");
        return ESP_OK;
    }

    cJSON *cmd = cJSON_GetObjectItem(root, "cmd");
    cJSON *val = cJSON_GetObjectItem(root, "value");
    if (cmd && cJSON_IsString(cmd) && val && cJSON_IsNumber(val)) {
        control_cmd_t c = {0};
        int v = (int)val->valuedouble;
        const char *cs = cmd->valuestring;
        global_state_lock();
        app_config_t *cfg = &global_state_get()->config;
        if (strcmp(cs, "relay") == 0) {
            c = (control_cmd_t){ .type = CMD_SET_RELAY, .value = v };
        } else if (strcmp(cs, "irfan") == 0) {
            c = (control_cmd_t){ .type = CMD_SET_IR_FAN, .value = v };
        } else if (strcmp(cs, "light") == 0) {
            c = (control_cmd_t){ .type = CMD_SET_LIGHT, .value = v };
        } else if (strcmp(cs, "auto") == 0) {
            cfg->auto_mode = v != 0;
            c.type = CMD_SET_MODE;
            c.value = (cfg->auto_mode ? 1 : 0) | (cfg->fan_auto ? 2 : 0) | (cfg->light_auto ? 4 : 0);
        } else if (strcmp(cs, "fanauto") == 0) {
            cfg->fan_auto = v != 0;
            c.type = CMD_SET_MODE;
            c.value = (cfg->auto_mode ? 1 : 0) | (cfg->fan_auto ? 2 : 0) | (cfg->light_auto ? 4 : 0);
        } else if (strcmp(cs, "lightauto") == 0) {
            cfg->light_auto = v != 0;
            c.type = CMD_SET_MODE;
            c.value = (cfg->auto_mode ? 1 : 0) | (cfg->fan_auto ? 2 : 0) | (cfg->light_auto ? 4 : 0);
        } else if (strcmp(cs, "learnir_on") == 0) {
            c = (control_cmd_t){ .type = CMD_LEARN_IR_ON };
        } else if (strcmp(cs, "learnir_off") == 0) {
            c = (control_cmd_t){ .type = CMD_LEARN_IR_OFF };
        } else if (strcmp(cs, "factory") == 0) {
            c = (control_cmd_t){ .type = CMD_FACTORY_RESET };
        }
        global_state_unlock();
        xQueueSend(cmd_queue, &c, pdMS_TO_TICKS(50));
    }

    cJSON_Delete(root);
    httpd_resp_send(req, "ok", 2);
    return ESP_OK;
}

static esp_err_t config_get_handler(httpd_req_t *req) {
    global_state_lock();
    app_config_t *cfg = &global_state_get()->config;
    cJSON *root = cJSON_CreateObject();
    cJSON_AddNumberToObject(root, "light_thr", cfg->light_thr_lux);
    cJSON_AddNumberToObject(root, "temp_thr", cfg->temp_thr_c);
    cJSON_AddNumberToObject(root, "hum_thr", cfg->hum_thr_pct);
    cJSON_AddStringToObject(root, "ssid", cfg->wifi_ssid);
    cJSON_AddStringToObject(root, "tuya_ip", cfg->tuya_ip);
    cJSON_AddStringToObject(root, "tuya_dev", cfg->tuya_dev_id);
    cJSON_AddStringToObject(root, "tuya_key", cfg->tuya_local_key);
    cJSON_AddNumberToObject(root, "tuya_dpid", cfg->tuya_dpid);
    cJSON_AddNumberToObject(root, "tuya_dpid2", cfg->tuya_dpid2);
    global_state_unlock();

    char *s = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, s, strlen(s));
    free(s);
    return ESP_OK;
}

static esp_err_t config_post_handler(httpd_req_t *req) {
    char buf[256];
    int ret = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (ret <= 0) return ESP_FAIL;
    buf[ret] = '\0';

    cJSON *root = cJSON_Parse(buf);
    if (!root) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad json");
        return ESP_OK;
    }

    global_state_lock();
    app_config_t *cfg = &global_state_get()->config;

    cJSON *j = cJSON_GetObjectItem(root, "light_thr");
    if (j) cfg->light_thr_lux = (float)j->valuedouble;
    j = cJSON_GetObjectItem(root, "temp_thr");
    if (j) cfg->temp_thr_c = (float)j->valuedouble;
    j = cJSON_GetObjectItem(root, "hum_thr");
    if (j) cfg->hum_thr_pct = (float)j->valuedouble;
    j = cJSON_GetObjectItem(root, "ssid");
    if (j && cJSON_IsString(j)) {
        strncpy(cfg->wifi_ssid, j->valuestring, sizeof(cfg->wifi_ssid) - 1);
        cfg->wifi_ssid[sizeof(cfg->wifi_ssid) - 1] = '\0';
    }
    j = cJSON_GetObjectItem(root, "pass");
    if (j && cJSON_IsString(j)) {
        strncpy(cfg->wifi_pass, j->valuestring, sizeof(cfg->wifi_pass) - 1);
        cfg->wifi_pass[sizeof(cfg->wifi_pass) - 1] = '\0';
    }
    j = cJSON_GetObjectItem(root, "tuya_ip");
    if (j && cJSON_IsString(j)) {
        strncpy(cfg->tuya_ip, j->valuestring, sizeof(cfg->tuya_ip) - 1);
        cfg->tuya_ip[sizeof(cfg->tuya_ip) - 1] = '\0';
    }
    j = cJSON_GetObjectItem(root, "tuya_dev");
    if (j && cJSON_IsString(j)) {
        strncpy(cfg->tuya_dev_id, j->valuestring, sizeof(cfg->tuya_dev_id) - 1);
        cfg->tuya_dev_id[sizeof(cfg->tuya_dev_id) - 1] = '\0';
    }
    j = cJSON_GetObjectItem(root, "tuya_key");
    if (j && cJSON_IsString(j)) {
        strncpy(cfg->tuya_local_key, j->valuestring, sizeof(cfg->tuya_local_key) - 1);
        cfg->tuya_local_key[sizeof(cfg->tuya_local_key) - 1] = '\0';
    }
    j = cJSON_GetObjectItem(root, "tuya_dpid");
    if (j && cJSON_IsNumber(j)) cfg->tuya_dpid = (uint8_t)j->valuedouble;
    j = cJSON_GetObjectItem(root, "tuya_dpid2");
    if (j && cJSON_IsNumber(j)) cfg->tuya_dpid2 = (uint8_t)j->valuedouble;

    app_config_save(cfg);
    global_state_unlock();
    cJSON_Delete(root);
    httpd_resp_send(req, "ok", 2);
    return ESP_OK;
}

static const httpd_uri_t uris[] = {
    { .uri = "/",                    .method = HTTP_GET,  .handler = root_get_handler },
    { .uri = "/api/status",          .method = HTTP_GET,  .handler = status_get_handler },
    { .uri = "/api/control",         .method = HTTP_POST, .handler = control_post_handler },
    { .uri = "/api/config",          .method = HTTP_POST, .handler = config_post_handler },
    { .uri = "/api/config",          .method = HTTP_GET,  .handler = config_get_handler },
    { .uri = "/laundary_room.svg",   .method = HTTP_GET,  .handler = icon_svg_get_handler },
    { .uri = "/manifest.json",       .method = HTTP_GET,  .handler = manifest_get_handler },
    { .uri = "/icon-192.png",        .method = HTTP_GET,  .handler = icon_192_get_handler },
    { .uri = "/icon-512.png",        .method = HTTP_GET,  .handler = icon_512_get_handler },
    { .uri = "/apple-touch-icon.png",.method = HTTP_GET,  .handler = apple_icon_get_handler },
    { .uri = "/favicon.ico",         .method = HTTP_GET,  .handler = favicon_get_handler },
};

esp_err_t http_server_init(void) {
    ESP_LOGI(TAG, "http_server init");
    return ESP_OK;
}

esp_err_t http_server_start(void) {
    if (s_server) return ESP_OK;

    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.server_port = 80;
    cfg.stack_size = 8192;
    cfg.max_uri_handlers = 16;

    if (httpd_start(&s_server, &cfg) != ESP_OK) {
        ESP_LOGE(TAG, "httpd_start failed");
        return ESP_FAIL;
    }

    for (size_t i = 0; i < sizeof(uris)/sizeof(uris[0]); i++) {
        httpd_register_uri_handler(s_server, &uris[i]);
    }

    /* mDNS so the dashboard is reachable at http://laundry_room.local */
    if (!s_mdns_started) {
        if (mdns_init() == ESP_OK) {
            mdns_hostname_set("laundry_room");
            mdns_instance_name_set("Laundry Room Controller");
            mdns_service_add(NULL, "_http", "_tcp", 80, NULL, 0);
            s_mdns_started = true;
        } else {
            ESP_LOGW(TAG, "mdns_init failed, hostname will not resolve");
        }
    }

    ESP_LOGI(TAG, "HTTP server started on port %d", cfg.server_port);
    return ESP_OK;
}
