#include "global_state.h"

#include "freertos/semphr.h"
#include <string.h>

static global_state_t s_state;
static SemaphoreHandle_t s_mutex;
QueueHandle_t cmd_queue = NULL;

global_state_t *global_state_get(void) {
    return &s_state;
}

void global_state_lock(void) {
    if (s_mutex) xSemaphoreTake(s_mutex, portMAX_DELAY);
}

void global_state_unlock(void) {
    if (s_mutex) xSemaphoreGive(s_mutex);
}

void global_state_copy(global_state_t *out) {
    if (!out) return;
    global_state_lock();
    memcpy(out, &s_state, sizeof(*out));
    global_state_unlock();
}

void global_state_status_snapshot(status_snapshot_t *out) {
    if (!out) return;
    global_state_lock();
    out->auto_mode          = s_state.config.auto_mode;
    out->fan_auto           = s_state.config.fan_auto;
    out->light_auto         = s_state.config.light_auto;
    out->fan_relay          = s_state.fan_relay;
    out->fan_ir             = s_state.fan_ir;
    out->light              = s_state.light;
    out->wifi_connected     = s_state.wifi_connected;
    out->tuya_connected     = s_state.tuya_connected;
    out->wifi_rssi          = s_state.wifi_rssi;
    memcpy(out->ip_addr, s_state.ip_addr, sizeof(out->ip_addr));
    strlcpy(out->wifi_ssid, s_state.config.wifi_ssid, sizeof(out->wifi_ssid));
    out->sensors_valid      = s_state.sensors.valid;
    out->aht_ok             = s_state.sensors.aht_ok;
    out->bmp_ok             = s_state.sensors.bmp_ok;
    out->veml_ok            = s_state.sensors.veml_ok;
    out->lux                = s_state.sensors.lux;
    out->temperature        = s_state.sensors.temperature;
    out->humidity           = s_state.sensors.humidity;
    out->pressure           = s_state.sensors.pressure;
    out->presence           = s_state.presence;
    out->ld2410_ok          = s_state.ld2410_ok;
    out->ld2410_rx_bytes    = s_state.ld2410_rx_bytes;
    out->ld2410_state       = s_state.ld2410_state;
    out->ld2410_std_mode    = s_state.ld2410_std_mode;
    out->moving_distance_cm = s_state.ld2410_moving_distance_cm;
    out->moving_energy      = s_state.ld2410_moving_energy;
    out->static_distance_cm = s_state.ld2410_static_distance_cm;
    out->static_energy      = s_state.ld2410_static_energy;
    global_state_unlock();
}

esp_err_t global_state_init(void) {
    s_mutex = xSemaphoreCreateMutex();
    if (!s_mutex) return ESP_ERR_NO_MEM;
    memset(&s_state, 0, sizeof(s_state));

    cmd_queue = xQueueCreate(16, sizeof(control_cmd_t));
    if (!cmd_queue) {
        vSemaphoreDelete(s_mutex);
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}
