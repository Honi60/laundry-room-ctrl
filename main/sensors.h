#pragma once

#include "esp_err.h"
#include <stdbool.h>

typedef struct {
    bool aht_ok;
    bool bmp_ok;
    bool veml_ok;
} sensor_health_t;

esp_err_t sensors_init(void);

/**
 * @brief Poll all sensors and update global_state.
 * @return ESP_OK if at least one sensor returned data.
 */
esp_err_t sensors_poll(sensor_health_t *health);

void sensors_start_task(void);

/* Pause/resume the periodic I2C re-probe (used while a diagnostic owns the
   I2C pins, e.g. pinpulse, so the driver doesn't grab them back). */
void sensors_set_paused(bool paused);

/* Last raw BMP280 conversion + its own temperature (diagnostics). */
void sensors_bmp_debug(int32_t *adc_p, int32_t *adc_t, float *temp);
