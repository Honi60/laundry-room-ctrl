#pragma once

#include "esp_err.h"

esp_err_t wifi_manager_init(void);
void      wifi_manager_reconnect(void);
