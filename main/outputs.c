#include "outputs.h"

#include "board.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "global_state.h"
#include "ir_ctrl.h"
#include "tuya_local.h"

#define TAG "outputs"

esp_err_t outputs_init(void) {
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << RELAY_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = 0,
        .pull_down_en = 0,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t ret = gpio_config(&io);
    outputs_set_relay(false);
    return ret;
}

static void set_relay_hw(bool on) {
    int level = (on ^ RELAY_ACTIVE_LOW) ? 1 : 0;
    gpio_set_level(RELAY_GPIO, level);
}

void outputs_set_relay(bool on) {
    set_relay_hw(on);
    global_state_lock();
    global_state_get()->fan_relay = on;
    global_state_unlock();
}

void outputs_set_ir_fan(bool on) {
    ir_ctrl_send(on);
    global_state_lock();
    global_state_get()->fan_ir = on;
    global_state_unlock();
    ESP_LOGI(TAG, "IR fan %s", on ? "on" : "off");
}

void outputs_set_light(bool on) {
    tuya_local_set_switch(on);
    global_state_lock();
    global_state_get()->light = on;
    global_state_unlock();
    ESP_LOGI(TAG, "light %s", on ? "on" : "off");
}
