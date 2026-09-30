#include "i2c_bus.h"

#include "board.h"
#include "esp_check.h"
#include "esp_log.h"

#define TAG "i2c_bus"

i2c_master_bus_handle_t i2c_bus = NULL;
SemaphoreHandle_t i2c_mutex = NULL;
int i2c_sda_gpio = -1, i2c_scl_gpio = -1;   /* pins the bus currently lives on */

static esp_err_t bus_create(int sda_gpio, int scl_gpio) {
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = -1,
        .sda_io_num = sda_gpio,
        .scl_io_num = scl_gpio,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        /* Must be 0: a non-zero depth enables the driver's asynchronous
           mode, whose ISR silently drops completion events for
           i2c_master_probe() (no device handle) -> every probe times out. */
        .trans_queue_depth = 0,
        .flags = {
            .enable_internal_pullup = 1,
        },
    };

    ESP_RETURN_ON_ERROR(i2c_new_master_bus(&bus_cfg, &i2c_bus), TAG, "i2c_new_master_bus");

    if (!i2c_mutex) i2c_mutex = xSemaphoreCreateRecursiveMutex();
    if (!i2c_mutex) {
        i2c_del_master_bus(i2c_bus);
        i2c_bus = NULL;
        return ESP_ERR_NO_MEM;
    }

    i2c_sda_gpio = sda_gpio;
    i2c_scl_gpio = scl_gpio;
    ESP_LOGI(TAG, "I2C bus ok, SDA=%d SCL=%d", sda_gpio, scl_gpio);
    return ESP_OK;
}

esp_err_t i2c_bus_init(void) {
    return bus_create(I2C_SDA_GPIO, I2C_SCL_GPIO);
}

esp_err_t i2c_bus_relocate(int sda_gpio, int scl_gpio) {
    xSemaphoreTakeRecursive(i2c_mutex, portMAX_DELAY);
    esp_err_t ret = i2c_del_master_bus(i2c_bus);
    if (ret == ESP_OK) {
        i2c_bus = NULL;
        ret = bus_create(sda_gpio, scl_gpio);
    }
    xSemaphoreGiveRecursive(i2c_mutex);
    return ret;
}
