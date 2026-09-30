#pragma once

#include "esp_err.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

extern i2c_master_bus_handle_t i2c_bus;
extern SemaphoreHandle_t i2c_mutex;
extern int i2c_sda_gpio, i2c_scl_gpio;

esp_err_t i2c_bus_init(void);

/* Tear down and re-create the bus on different pins (diagnostic — used to
   test whether a pin is physically dead). Any device handles created on the
   old bus are stale afterwards; sensors re-probe and re-add themselves. */
esp_err_t i2c_bus_relocate(int sda_gpio, int scl_gpio);
