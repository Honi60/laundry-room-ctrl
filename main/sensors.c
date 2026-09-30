#include "sensors.h"

#include "board.h"
#include "i2c_bus.h"
#include "global_state.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include <string.h>

#define TAG          "sensors"
#define I2C_TIMEOUT  100

#define AHT_ADDR     0x38
#define VEML_ADDR    0x10

/* VEML7700 tables from MySmartClock */
static const uint8_t VEML_GAIN_CODES[4] = {0x00, 0x01, 0x02, 0x03}; /* x1, x2, x1/8, x1/4 */
static const uint8_t VEML_IT_CODES[6]   = {0x0C, 0x08, 0x00, 0x01, 0x02, 0x03}; /* 25,50,100,200,400,800 ms */
static const float VEML_RESOLUTION[4][6] = {
    {0.2304f, 0.1152f, 0.0576f, 0.0288f, 0.0144f, 0.0072f},
    {0.1152f, 0.0576f, 0.0288f, 0.0144f, 0.0072f, 0.0036f},
    {1.8432f, 0.9216f, 0.4608f, 0.2304f, 0.1152f, 0.0576f},
    {0.9216f, 0.4608f, 0.2304f, 0.1152f, 0.0576f, 0.0288f}
};

static i2c_master_dev_handle_t s_aht = NULL;
static i2c_master_dev_handle_t s_bmp = NULL;
static i2c_master_dev_handle_t s_veml = NULL;

static uint8_t s_bmp_addr = 0;
static uint8_t s_veml_gain = 0;  /* x1 */
static uint8_t s_veml_it = 2;    /* 100 ms */

static bool aht_init(void) {
    i2c_device_config_t dev = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = AHT_ADDR,
        .scl_speed_hz = I2C_FREQ_HZ,
        .scl_wait_us = 0,
    };
    if (i2c_master_bus_add_device(i2c_bus, &dev, &s_aht) != ESP_OK) return false;

    if (i2c_master_probe(i2c_bus, AHT_ADDR, I2C_TIMEOUT) != ESP_OK) {
        i2c_master_bus_rm_device(s_aht);
        s_aht = NULL;
        return false;
    }

    uint8_t rst = 0xBA;
    if (i2c_master_transmit(s_aht, &rst, 1, I2C_TIMEOUT) != ESP_OK) {
        i2c_master_bus_rm_device(s_aht);
        s_aht = NULL;
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(20));

    uint8_t initCmd[3] = {0xBE, 0x08, 0x00};
    if (i2c_master_transmit(s_aht, initCmd, 3, I2C_TIMEOUT) != ESP_OK) {
        i2c_master_bus_rm_device(s_aht);
        s_aht = NULL;
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
    return true;
}

static bool aht_read(float *tempC, float *humPct) {
    uint8_t trig[3] = {0xAC, 0x33, 0x00};
    if (i2c_master_transmit(s_aht, trig, 3, I2C_TIMEOUT) != ESP_OK) return false;

    vTaskDelay(pdMS_TO_TICKS(85));

    uint8_t d[6];
    if (i2c_master_receive(s_aht, d, 6, I2C_TIMEOUT) != ESP_OK) return false;

    if (d[0] & 0x80) return false; /* busy */

    uint32_t rawHum  = ((uint32_t)d[1] << 12) | ((uint32_t)d[2] << 4) | (d[3] >> 4);
    uint32_t rawTemp = (((uint32_t)d[3] & 0x0F) << 16) | ((uint32_t)d[4] << 8) | d[5];

    if (humPct) *humPct = (rawHum * 100.0f) / 1048576.0f;
    if (tempC)  *tempC  = (rawTemp * 200.0f) / 1048576.0f - 50.0f;
    return true;
}

static uint8_t bmp280_find_addr(void) {
    if (i2c_master_probe(i2c_bus, 0x76, I2C_TIMEOUT) == ESP_OK) return 0x76;
    if (i2c_master_probe(i2c_bus, 0x77, I2C_TIMEOUT) == ESP_OK) return 0x77;
    return 0;
}

static bool bmp280_init(void) {
    s_bmp_addr = bmp280_find_addr();
    if (!s_bmp_addr) return false;

    i2c_device_config_t dev = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = s_bmp_addr,
        .scl_speed_hz = I2C_FREQ_HZ,
    };
    if (i2c_master_bus_add_device(i2c_bus, &dev, &s_bmp) != ESP_OK) return false;

    uint8_t reg = 0xD0, chipId = 0;
    if (i2c_master_transmit_receive(s_bmp, &reg, 1, &chipId, 1, I2C_TIMEOUT) != ESP_OK ||
        (chipId != 0x58 && chipId != 0x60)) {
        i2c_master_bus_rm_device(s_bmp);
        s_bmp = NULL;
        return false;
    }

    /* Normal mode, x1 oversampling. If this write fails the sensor stays
       asleep and 0xF7 returns its reset value 0x80000 forever, which decodes
       to a plausible-looking constant pressure - so verify it took. */
    uint8_t ctrl[2] = {0xF4, 0x27};
    uint8_t cfg[2] = {0xF5, 0x00};
    uint8_t ctrl_reg = 0xF4, readback = 0;
    if (i2c_master_transmit(s_bmp, ctrl, 2, I2C_TIMEOUT) != ESP_OK ||
        i2c_master_transmit(s_bmp, cfg, 2, I2C_TIMEOUT) != ESP_OK ||
        i2c_master_transmit_receive(s_bmp, &ctrl_reg, 1, &readback, 1, I2C_TIMEOUT) != ESP_OK ||
        readback != 0x27) {
        ESP_LOGW(TAG, "BMP280 ctrl write failed (0xF4=0x%02X)", readback);
        i2c_master_bus_rm_device(s_bmp);
        s_bmp = NULL;
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
    return true;
}

/* Last raw BMP280 conversion, for diagnostics. */
static int32_t s_bmp_adc_p = 0, s_bmp_adc_t = 0;
static float   s_bmp_temp = 0;

void sensors_bmp_debug(int32_t *adc_p, int32_t *adc_t, float *temp) {
    if (adc_p) *adc_p = s_bmp_adc_p;
    if (adc_t) *adc_t = s_bmp_adc_t;
    if (temp)  *temp  = s_bmp_temp;
}

static bool bmp280_read(float *tempC, float *pressHpa) {
    uint8_t reg = 0x88;
    uint8_t calib[24];
    if (i2c_master_transmit_receive(s_bmp, &reg, 1, calib, 24, I2C_TIMEOUT) != ESP_OK) return false;

    uint16_t dig_T1 = (calib[1] << 8) | calib[0];
    int16_t  dig_T2 = (calib[3] << 8) | calib[2];
    int16_t  dig_T3 = (calib[5] << 8) | calib[4];
    uint16_t dig_P1 = (calib[7] << 8) | calib[6];
    int16_t  dig_P2 = (calib[9] << 8) | calib[8];
    int16_t  dig_P3 = (calib[11] << 8) | calib[10];
    int16_t  dig_P4 = (calib[13] << 8) | calib[12];
    int16_t  dig_P5 = (calib[15] << 8) | calib[14];
    int16_t  dig_P6 = (calib[17] << 8) | calib[16];
    int16_t  dig_P7 = (calib[19] << 8) | calib[18];
    int16_t  dig_P8 = (calib[21] << 8) | calib[20];
    int16_t  dig_P9 = (calib[23] << 8) | calib[22];

    reg = 0xF7;
    uint8_t raw[6];
    if (i2c_master_transmit_receive(s_bmp, &reg, 1, raw, 6, I2C_TIMEOUT) != ESP_OK) return false;

    int32_t adc_P = ((uint32_t)raw[0] << 12) | ((uint32_t)raw[1] << 4) | (raw[2] >> 4);
    int32_t adc_T = ((uint32_t)raw[3] << 12) | ((uint32_t)raw[4] << 4) | (raw[5] >> 4);
    s_bmp_adc_p = adc_P;
    s_bmp_adc_t = adc_T;

    int32_t var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
    int32_t var2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) * ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;
    int32_t t_fine = var1 + var2;
    float T = ((t_fine * 5 + 128) >> 8) / 100.0f;

    int64_t v1 = ((int64_t)t_fine) - 128000;
    int64_t v2 = v1 * v1 * (int64_t)dig_P6;
    v2 = v2 + ((v1 * (int64_t)dig_P5) << 17);
    v2 = v2 + (((int64_t)dig_P4) << 35);
    v1 = ((v1 * v1 * (int64_t)dig_P3) >> 8) + ((v1 * (int64_t)dig_P2) << 12);
    v1 = (((((int64_t)1) << 47) + v1)) * ((int64_t)dig_P1) >> 33;
    float P = 0;
    if (v1 != 0) {
        int64_t p = 1048576 - adc_P;
        p = (((p << 31) - v2) * 3125) / v1;
        v1 = (((int64_t)dig_P9) * (p >> 13) * (p >> 13)) >> 25;
        v2 = (((int64_t)dig_P8) * p) >> 19;
        p = ((p + v1 + v2) >> 8) + (((int64_t)dig_P7) << 4);
        P = (float)p / 256.0f / 100.0f; /* Pa -> hPa */
    }

    s_bmp_temp = T;
    if (tempC) *tempC = T;
    if (pressHpa) *pressHpa = P;
    return true;
}

static bool veml_init(void) {
    i2c_device_config_t dev = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = VEML_ADDR,
        .scl_speed_hz = I2C_FREQ_HZ,
    };
    if (i2c_master_bus_add_device(i2c_bus, &dev, &s_veml) != ESP_OK) return false;

    if (i2c_master_probe(i2c_bus, VEML_ADDR, I2C_TIMEOUT) != ESP_OK) {
        i2c_master_bus_rm_device(s_veml);
        s_veml = NULL;
        return false;
    }

    uint16_t conf = ((uint16_t)VEML_GAIN_CODES[s_veml_gain] << 11) |
                    ((uint16_t)VEML_IT_CODES[s_veml_it] << 6);
    uint8_t cfg[3] = {0x00, (uint8_t)(conf & 0xFF), (uint8_t)(conf >> 8)};
    if (i2c_master_transmit(s_veml, cfg, 3, I2C_TIMEOUT) != ESP_OK) {
        i2c_master_bus_rm_device(s_veml);
        s_veml = NULL;
        return false;
    }
    return true;
}

static bool veml_read(float *lux) {
    uint8_t reg = 0x04;
    uint8_t d[2];
    if (i2c_master_transmit_receive(s_veml, &reg, 1, d, 2, I2C_TIMEOUT) != ESP_OK) return false;

    uint16_t c = (d[1] << 8) | d[0];
    if (lux) *lux = c * VEML_RESOLUTION[s_veml_gain][s_veml_it];
    return true;
}

esp_err_t sensors_init(void) {
    ESP_LOGI(TAG, "initialising I2C sensors...");

    /* Re-probe runs every 5s; keep driver errors visible (they distinguish
       software vs hardware timeouts) but drop the chatty warnings. */
    esp_log_level_set("i2c.master", ESP_LOG_ERROR);

    xSemaphoreTakeRecursive(i2c_mutex, portMAX_DELAY);
    /* Flush any slave left mid-transaction before the first probe. */
    i2c_master_bus_reset(i2c_bus);
    bool aht_ok = aht_init();
    bool bmp_ok = bmp280_init();
    bool veml_ok = veml_init();
    xSemaphoreGiveRecursive(i2c_mutex);

    ESP_LOGI(TAG, "sensors: AHT20=%s BMP280=%s VEML7700=%s",
             aht_ok ? "ok" : "missing", bmp_ok ? "ok" : "missing", veml_ok ? "ok" : "missing");
    return ESP_OK;
}

#define SENSOR_REPROBE_MS 5000
#define SENSOR_FAIL_LIMIT  3

static TickType_t s_last_reprobe = 0;
static uint8_t s_aht_fail = 0, s_bmp_fail = 0, s_veml_fail = 0;
static volatile bool s_paused = false;

void sensors_set_paused(bool paused) { s_paused = paused; }

/* Remove a dead handle so the periodic re-probe can re-detect the sensor. */
static void drop_if_failed(bool ok, uint8_t *fails,
                           i2c_master_dev_handle_t *dev) {
    if (ok) {
        *fails = 0;
    } else if (dev && *dev && ++(*fails) >= SENSOR_FAIL_LIMIT) {
        i2c_master_bus_rm_device(*dev);
        *dev = NULL;
        *fails = 0;
    }
}

esp_err_t sensors_poll(sensor_health_t *health) {
    if (health) memset(health, 0, sizeof(*health));
    if (s_paused) return ESP_FAIL;

    xSemaphoreTakeRecursive(i2c_mutex, portMAX_DELAY);

    /* Re-probe missing sensors periodically so hot-plugged hardware works.
       A slave left mid-transaction can hold the bus (all probes then time
       out) — clock SCL first to flush it, which is what un-sticks the AHT20. */
    if ((!s_aht || !s_bmp || !s_veml) &&
        (xTaskGetTickCount() - s_last_reprobe) > pdMS_TO_TICKS(SENSOR_REPROBE_MS)) {
        s_last_reprobe = xTaskGetTickCount();
        i2c_master_bus_reset(i2c_bus);
        if (!s_aht && aht_init())  ESP_LOGI(TAG, "AHT20 detected");
        if (!s_bmp && bmp280_init()) ESP_LOGI(TAG, "BMP280 detected");
        if (!s_veml && veml_init()) ESP_LOGI(TAG, "VEML7700 detected");
    }

    float t, h, p, lux;
    bool aht_ok = false, bmp_ok = false, veml_ok = false;

    if (s_aht) aht_ok = aht_read(&t, &h);
    drop_if_failed(aht_ok, &s_aht_fail, &s_aht);
    if (aht_ok) {
        global_state_lock();
        global_state_get()->sensors.temperature = t;
        global_state_get()->sensors.humidity = h;
        global_state_unlock();
    }

    if (s_bmp) bmp_ok = bmp280_read(&t, &p);
    drop_if_failed(bmp_ok, &s_bmp_fail, &s_bmp);
    if (bmp_ok) {
        global_state_lock();
        /* Prefer AHT temp if available, else BMP temp */
        if (!aht_ok) global_state_get()->sensors.temperature = t;
        global_state_get()->sensors.pressure = p;
        global_state_unlock();
    }

    if (s_veml) veml_ok = veml_read(&lux);
    drop_if_failed(veml_ok, &s_veml_fail, &s_veml);
    if (veml_ok) {
        global_state_lock();
        global_state_get()->sensors.lux = lux;
        global_state_unlock();
    }

    xSemaphoreGiveRecursive(i2c_mutex);

    global_state_lock();
    global_state_t *st = global_state_get();
    st->sensors.aht_ok = aht_ok;
    st->sensors.bmp_ok = bmp_ok;
    st->sensors.veml_ok = veml_ok;
    if (aht_ok || bmp_ok || veml_ok) {
        st->sensors.valid = true;
    }
    global_state_unlock();

    if (health) {
        health->aht_ok = aht_ok;
        health->bmp_ok = bmp_ok;
        health->veml_ok = veml_ok;
    }

    return (aht_ok || bmp_ok || veml_ok) ? ESP_OK : ESP_FAIL;
}

static void sensor_task(void *arg) {
    (void)arg;
    for (;;) {
        sensors_poll(NULL);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void sensors_start_task(void) {
    xTaskCreatePinnedToCore(sensor_task, "sensor_task", 4096, NULL, 5, NULL, 1);
}
