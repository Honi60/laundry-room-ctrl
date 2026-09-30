#include "bt_spp.h"

#include "cli.h"
#include "status_led.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_bt_device.h"
#include "esp_spp_api.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include <stdlib.h>
#include <string.h>

#define TAG "bt_spp"
#define DEVICE_NAME BT_DEVICE_NAME
#define SPP_SERVER_NAME "Laundry SPP"
#define BT_RX_LINE_MAX 128

static volatile uint32_t s_spp_handle = 0;
static volatile bool s_connected = false;
static volatile bool s_congested = false;
static SemaphoreHandle_t s_cong_sem;
static QueueHandle_t s_line_queue;

/* RX line accumulator (SPP data arrives in arbitrary chunks, not lines) */
static char s_rx_buf[BT_RX_LINE_MAX];
static size_t s_rx_len = 0;

static void bt_output(const char *data, size_t len, void *user) {
    (void)user;
    if (!s_connected || !s_spp_handle) return;

    for (int i = 0; i < 30; i++) {
        esp_spp_status_t r = esp_spp_write(s_spp_handle, len, (uint8_t *)data);
        if (r == ESP_SPP_SUCCESS) return;
        if (r != ESP_SPP_BUSY && r != ESP_SPP_NO_RESOURCE && r != ESP_SPP_FAILURE) return;
        /* Congested - wait for ESP_SPP_CONG_EVT to signal the link drained */
        if (xSemaphoreTake(s_cong_sem, pdMS_TO_TICKS(100)) != pdTRUE) {
            if (!s_congested) continue;   /* decongested without an event */
            return;                        /* still congested - drop */
        }
    }
}

static void bt_cli_task(void *arg) {
    (void)arg;
    char *line = NULL;
    for (;;) {
        if (xQueueReceive(s_line_queue, &line, portMAX_DELAY) == pdTRUE && line) {
            cli_process_line(line, bt_output, NULL);
            free(line);
            line = NULL;
        }
    }
}

static void rx_push_line(void) {
    char *line = malloc(s_rx_len + 1);
    if (line) {
        memcpy(line, s_rx_buf, s_rx_len);
        line[s_rx_len] = '\0';
        if (xQueueSend(s_line_queue, &line, 0) != pdTRUE) free(line);
    }
    s_rx_len = 0;
}

static void rx_feed(const uint8_t *data, uint16_t len) {
    for (uint16_t i = 0; i < len; i++) {
        char c = (char)data[i];
        if (c == '\r' || c == '\n') {
            if (s_rx_len > 0) rx_push_line();
        } else if (c >= 32 && s_rx_len < sizeof(s_rx_buf) - 1) {
            s_rx_buf[s_rx_len++] = c;
        } else if (s_rx_len >= sizeof(s_rx_buf) - 1) {
            s_rx_len = 0;   /* overflow - drop the line */
        }
    }
}

static void esp_spp_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param) {
    switch (event) {
    case ESP_SPP_INIT_EVT:
        if (param->init.status == ESP_SPP_SUCCESS) {
            esp_spp_start_srv(ESP_SPP_SEC_NONE, ESP_SPP_ROLE_SLAVE, 0, SPP_SERVER_NAME);
        } else {
            ESP_LOGE(TAG, "SPP init failed, status=%d", param->init.status);
        }
        break;
    case ESP_SPP_START_EVT:
        if (param->start.status == ESP_SPP_SUCCESS) {
            ESP_LOGI(TAG, "SPP server up, discoverable as '%s'", DEVICE_NAME);
            esp_bt_gap_set_device_name(DEVICE_NAME);
            esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
            status_led_set_ready(true);
        } else {
            ESP_LOGE(TAG, "SPP server start failed, status=%d", param->start.status);
        }
        break;
    case ESP_SPP_DATA_IND_EVT:
        rx_feed(param->data_ind.data, param->data_ind.len);
        break;
    case ESP_SPP_CONG_EVT:
        s_congested = param->cong.cong;
        if (!s_congested) xSemaphoreGive(s_cong_sem);
        break;
    case ESP_SPP_SRV_OPEN_EVT:
        s_spp_handle = param->srv_open.handle;
        s_connected = true;
        s_rx_len = 0;
        ESP_LOGI(TAG, "BT client connected");
        break;
    case ESP_SPP_CLOSE_EVT:
        s_connected = false;
        s_spp_handle = 0;
        s_congested = false;
        xSemaphoreGive(s_cong_sem);   /* wake any pending writer */
        break;
    default:
        break;
    }
}

static void esp_bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param) {
    (void)param;
    if (event == ESP_BT_GAP_AUTH_CMPL_EVT) {
        if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
            ESP_LOGI(TAG, "BT auth success");
        } else {
            ESP_LOGE(TAG, "BT auth failed, status:%d", param->auth_cmpl.stat);
        }
    }
}

esp_err_t bt_spp_init(void) {
    ESP_LOGI(TAG, "bluetooth init");

    s_cong_sem = xSemaphoreCreateBinary();
    s_line_queue = xQueueCreate(8, sizeof(char *));
    if (!s_cong_sem || !s_line_queue) return ESP_ERR_NO_MEM;

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_RETURN_ON_ERROR(esp_bt_controller_init(&bt_cfg), TAG, "controller init");
    ESP_RETURN_ON_ERROR(esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT), TAG, "controller enable");

    esp_bluedroid_config_t bluedroid_cfg = BT_BLUEDROID_INIT_CONFIG_DEFAULT();
    ESP_RETURN_ON_ERROR(esp_bluedroid_init_with_cfg(&bluedroid_cfg), TAG, "bluedroid init");
    ESP_RETURN_ON_ERROR(esp_bluedroid_enable(), TAG, "bluedroid enable");

    ESP_RETURN_ON_ERROR(esp_bt_gap_register_callback(esp_bt_gap_cb), TAG, "gap cb");
    ESP_RETURN_ON_ERROR(esp_spp_register_callback(esp_spp_cb), TAG, "spp cb");

    /* Set name + discoverable early so the device is visible even before
       the SPP server finishes starting (re-set again on SPP_START_EVT). */
    esp_bt_gap_set_device_name(DEVICE_NAME);
    esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);

    esp_spp_cfg_t bt_spp_cfg = {
        .mode = ESP_SPP_MODE_CB,
        .enable_l2cap_ertm = true,
        .tx_buffer_size = 0,
    };
    ESP_RETURN_ON_ERROR(esp_spp_enhanced_init(&bt_spp_cfg), TAG, "spp init");

    xTaskCreatePinnedToCore(bt_cli_task, "bt_cli", 4096, NULL, 3, NULL, 0);
    return ESP_OK;
}

bool bt_spp_is_connected(void) {
    return s_connected;
}
