#include "board.h"
#include "config_manager.h"
#include "global_state.h"
#include "status_led.h"
#include "i2c_bus.h"
#include "sensors.h"
#include "ld2410_hal.h"
#include "ir_ctrl.h"
#include "tuya_local.h"
#include "outputs.h"
#include "state_machine.h"
#include "wifi_manager.h"
#include "http_server.h"
#include "bt_spp.h"
#include "cli.h"

#include "esp_log.h"
#include "nvs_flash.h"

#define TAG "main"

void app_main(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    ESP_LOGI(TAG, "Laundry Room Controller booting");

    ESP_ERROR_CHECK(app_config_init());
    ESP_ERROR_CHECK(global_state_init());
    ESP_ERROR_CHECK(status_led_init());

    global_state_t *st = global_state_get();
    ESP_ERROR_CHECK(app_config_load(&st->config));

    ESP_ERROR_CHECK(i2c_bus_init());
    ESP_ERROR_CHECK(ir_ctrl_init());
    ESP_ERROR_CHECK(tuya_local_init());
    ESP_ERROR_CHECK(outputs_init());
    ESP_ERROR_CHECK(sensors_init());
    ESP_ERROR_CHECK(ld2410_hal_init());
    ESP_ERROR_CHECK(state_machine_init());
    ESP_ERROR_CHECK(wifi_manager_init());
    ESP_ERROR_CHECK(http_server_init());
    ESP_ERROR_CHECK(bt_spp_init());
    ESP_ERROR_CHECK(cli_init());

    sensors_start_task();
    ld2410_hal_start_task();
    state_machine_start_task();

    status_led_set_boot(true);
}
