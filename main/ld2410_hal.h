#pragma once

#include <stdbool.h>
#include <stddef.h>

#include "esp_err.h"

esp_err_t ld2410_hal_init(void);
void      ld2410_hal_start_task(void);

/* Send config-mode commands to set the "unmanned delay" (seconds, 10-120)
   and fast response speed. Requires the ESP->sensor TX wire. */
esp_err_t ld2410_set_unmanned_delay(uint32_t seconds);

/* Enter config mode and read firmware version + serial number.
   Buffers may be NULL to skip that item. */
esp_err_t ld2410_read_info(char *ver_buf, size_t ver_sz,
                           char *sn_buf, size_t sn_sz);

/* Switch report output: standard (F4..F5 frames w/ per-gate energy)
   or minimal (6E..62). Returns ESP_OK when the sensor ACKs it. */
esp_err_t ld2410_set_output_mode(bool standard);

/* Read common params (0x0071) inside a config session.
   vals[0..5] = far gate, near gate, unmanned delay s,
   status Hz*10, distance Hz*10, response speed.
   If frame/frame_len are non-NULL the complete 0x0171 ACK wire
   frame (header..tail) is copied out. */
esp_err_t ld2410_read_params(uint32_t vals[6],
                             uint8_t *frame, uint16_t *frame_len);

/* Raw wire test: enable cfg -> write SN -> read SN -> end cfg.
   Logs every TX and RX byte in hex. */
void ld2410_exchange_test(void);

/* Hex-dump the next `count` report frames the sensor sends. */
void ld2410_dump_request(int count);

/* Send an arbitrary command word with raw parameter bytes inside one
   enable/end config session. Logs every TX and RX byte in hex. */
esp_err_t ld2410_raw_cmd(uint16_t cmd, const uint8_t *val, uint16_t vlen);
