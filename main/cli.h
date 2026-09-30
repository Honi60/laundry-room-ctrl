#pragma once

#include "esp_err.h"
#include <stddef.h>

/* Output callback for one command. */
typedef void (*cli_output_t)(const char *data, size_t len, void *user);

esp_err_t cli_init(void);

/**
 * @brief Process one line of text and produce response.
 *
 * @param line   Null-terminated command line.
 * @param out    Optional output callback. If NULL, output is sent to serial (stdout).
 * @param user   User context for the output callback.
 */
esp_err_t cli_process_line(const char *line, cli_output_t out, void *user);
