#pragma once

#include "esp_err.h"
#include <stdbool.h>

esp_err_t status_led_init(void);
void      status_led_set_boot(bool boot_done);
void      status_led_set_wifi_connected(bool connected);
void      status_led_set_ready(bool ready);   /* system is ready (BT/Wi-Fi up) */
void      status_led_set_fault(bool fault);
void      status_led_force_state(bool on);
