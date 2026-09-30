#pragma once

#include "esp_err.h"
#include <stdbool.h>

esp_err_t outputs_init(void);

/* Setters return immediately; the output state is protected by global_state. */
void outputs_set_relay(bool on);
void outputs_set_ir_fan(bool on);
void outputs_set_light(bool on);
