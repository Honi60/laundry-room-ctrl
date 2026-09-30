#pragma once

#include "esp_err.h"
#include <stdbool.h>

#define BT_DEVICE_NAME "LaundryCtrl"

esp_err_t bt_spp_init(void);
bool bt_spp_is_connected(void);
