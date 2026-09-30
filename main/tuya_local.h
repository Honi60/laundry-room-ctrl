#pragma once

#include "esp_err.h"
#include <stdbool.h>

esp_err_t tuya_local_init(void);
esp_err_t tuya_local_set_switch(bool on);
