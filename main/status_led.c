#include "status_led.h"

#include "board.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TAG "status_led"

#define LED_ON()  gpio_set_level(STATUS_LED_GPIO, STATUS_LED_ACTIVE_LOW ? 0 : 1)
#define LED_OFF() gpio_set_level(STATUS_LED_GPIO, STATUS_LED_ACTIVE_LOW ? 1 : 0)

static volatile bool s_boot_done;
static volatile bool s_connected;
static volatile bool s_fault;
static volatile bool s_forced;
static volatile bool s_forced_state;

static void led_task(void *arg) {
    (void)arg;
    for (;;) {
        if (s_forced) {
            if (s_forced_state) LED_ON(); else LED_OFF();
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }
        if (s_fault) {
            LED_ON();
            vTaskDelay(pdMS_TO_TICKS(100));
            LED_OFF();
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        if (!s_boot_done || !s_connected) {
            /* fast blink 0.2 s */
            LED_ON();
            vTaskDelay(pdMS_TO_TICKS(100));
            LED_OFF();
            vTaskDelay(pdMS_TO_TICKS(100));
        } else {
            /* slow blink 1 s */
            LED_ON();
            vTaskDelay(pdMS_TO_TICKS(50));
            LED_OFF();
            vTaskDelay(pdMS_TO_TICKS(950));
        }
    }
}

esp_err_t status_led_init(void) {
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << STATUS_LED_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = 0,
        .pull_down_en = 0,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t ret = gpio_config(&io);
    LED_OFF();

    s_boot_done = false;
    s_connected = false;
    s_fault = false;
    s_forced = false;

    if (ret == ESP_OK) {
        xTaskCreatePinnedToCore(led_task, "led_task", 2048, NULL, 2, NULL, 0);
    }
    return ret;
}

void status_led_set_boot(bool boot_done) {
    s_boot_done = boot_done;
}

void status_led_set_wifi_connected(bool connected) {
    s_connected = connected;
}

void status_led_set_ready(bool ready) {
    s_connected = ready;
}

void status_led_set_fault(bool fault) {
    s_fault = fault;
}

void status_led_force_state(bool on) {
    s_forced = true;
    s_forced_state = on;
}
