#include "wifi_manager.h"

#include "config_manager.h"
#include "global_state.h"
#include "http_server.h"
#include "status_led.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include <string.h>
#include <stdlib.h>

#define TAG "wifi_manager"

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static EventGroupHandle_t s_wifi_ev;
static int s_retry_count = 0;
static volatile bool s_reconnect_requested = false;

static void start_wifi_from_config(void);

static void event_handler(void *arg, esp_event_base_t event_base,
                          int32_t event_id, void *event_data) {
    (void)arg;
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        if (!s_reconnect_requested) {
            start_wifi_from_config();
        }
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_event_sta_disconnected_t *d = (wifi_event_sta_disconnected_t *)event_data;
        ESP_LOGW(TAG, "disconnected, reason=%d", d->reason);
        global_state_lock();
        global_state_get()->wifi_connected = false;
        global_state_get()->wifi_rssi = 0;
        global_state_get()->ip_addr[0] = '\0';
        global_state_unlock();

        if (s_reconnect_requested) {
            s_reconnect_requested = false;
            start_wifi_from_config();
            return;
        }

        if (s_retry_count < 5) {
            esp_wifi_connect();
            s_retry_count++;
            ESP_LOGI(TAG, "retry %d", s_retry_count);
        } else {
            xEventGroupSetBits(s_wifi_ev, WIFI_FAIL_BIT);
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *e = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "got ip " IPSTR, IP2STR(&e->ip_info.ip));
        global_state_lock();
        snprintf(global_state_get()->ip_addr, sizeof(global_state_get()->ip_addr),
                 IPSTR, IP2STR(&e->ip_info.ip));
        global_state_get()->wifi_connected = true;
        global_state_unlock();
        status_led_set_wifi_connected(true);
        xEventGroupSetBits(s_wifi_ev, WIFI_CONNECTED_BIT);
        s_retry_count = 0;

        http_server_start();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_SCAN_DONE) {
        uint16_t n = 0;
        esp_wifi_scan_get_ap_num(&n);
        if (n == 0) {
            ESP_LOGW(TAG, "no APs found");
            return;
        }
        wifi_ap_record_t *aps = malloc(sizeof(wifi_ap_record_t) * n);
        if (!aps) return;
        esp_wifi_scan_get_ap_records(&n, aps);

        global_state_t *st = global_state_get();
        const char *ssid = (const char *)st->config.wifi_ssid;
        int best = -1;
        for (int i = 0; i < n; i++) {
            ESP_LOGI(TAG, "scan: %s rssi=%d", aps[i].ssid, aps[i].rssi);
            if (strcmp((const char *)aps[i].ssid, ssid) == 0) {
                if (best < 0 || aps[i].rssi > aps[best].rssi) best = i;
            }
        }

        if (best >= 0) {
            wifi_config_t cfg = {0};
            strncpy((char *)cfg.sta.ssid, (const char *)aps[best].ssid, sizeof(cfg.sta.ssid) - 1);
            cfg.sta.ssid[sizeof(cfg.sta.ssid) - 1] = '\0';
            global_state_lock();
            strncpy((char *)cfg.sta.password, st->config.wifi_pass, sizeof(cfg.sta.password) - 1);
            cfg.sta.password[sizeof(cfg.sta.password) - 1] = '\0';
            global_state_unlock();
            esp_wifi_set_config(WIFI_IF_STA, &cfg);
            esp_wifi_connect();
            ESP_LOGI(TAG, "connecting to %s", aps[best].ssid);
        } else {
            ESP_LOGW(TAG, "SSID '%s' not found in scan", ssid);
        }
        free(aps);
    }
}

static void wifi_task(void *arg) {
    (void)arg;
    for (;;) {
        EventBits_t bits = xEventGroupWaitBits(s_wifi_ev,
                                               WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                               pdTRUE, pdFALSE, portMAX_DELAY);
        if (bits & WIFI_CONNECTED_BIT) {
            ESP_LOGI(TAG, "connected");
        } else if (bits & WIFI_FAIL_BIT) {
            ESP_LOGE(TAG, "failed to connect after retries");
            vTaskDelay(pdMS_TO_TICKS(30000));
            s_retry_count = 0;
            esp_wifi_connect();
        }
    }
}

static void start_wifi_from_config(void) {
    global_state_t *st = global_state_get();
    if (st->config.wifi_ssid[0] == '\0') {
        ESP_LOGW(TAG, "no Wi-Fi credentials, staying disconnected");
        return;
    }

    wifi_scan_config_t scan = {0};
    ESP_LOGI(TAG, "scanning for SSID '%s'", st->config.wifi_ssid);
    if (esp_wifi_scan_start(&scan, false) != ESP_OK) {
        ESP_LOGW(TAG, "scan start failed");
    }
}

esp_err_t wifi_manager_init(void) {
    s_wifi_ev = xEventGroupCreate();
    if (!s_wifi_ev) return ESP_ERR_NO_MEM;

    ESP_RETURN_ON_ERROR(esp_netif_init(), TAG, "netif init");
    ESP_RETURN_ON_ERROR(esp_event_loop_create_default(), TAG, "event loop");
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_RETURN_ON_ERROR(esp_wifi_init(&cfg), TAG, "wifi init");

    esp_event_handler_instance_t any_id;
    esp_event_handler_instance_t got_ip;
    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL, &any_id);
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL, &got_ip);

    ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_STA), TAG, "set mode");
    ESP_RETURN_ON_ERROR(esp_wifi_start(), TAG, "wifi start");

    xTaskCreatePinnedToCore(wifi_task, "wifi_task", 4096, NULL, 4, NULL, 0);
    return ESP_OK;
}

void wifi_manager_reconnect(void)
{
    ESP_LOGI(TAG, "reconnect requested");
    s_reconnect_requested = true;
    if (esp_wifi_disconnect() != ESP_OK) {
        /* Not connected - no DISCONNECTED event will arrive,
           so start the connection attempt directly. */
        s_reconnect_requested = false;
        start_wifi_from_config();
    }
}
