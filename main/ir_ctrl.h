#pragma once

#include "esp_err.h"
#include <stdbool.h>

esp_err_t ir_ctrl_init(void);

/* Re-apply the TX carrier frequency at runtime (scope debugging). */
esp_err_t ir_ctrl_set_carrier(uint32_t freq_hz);

/* Send the learned ON or OFF IR burst. */
void ir_ctrl_send(bool on);

/* Send a named fan remote command: "on", "off", "timer", "mode", "swing". */
void ir_ctrl_send_fan(const char *cmd);

/* Transmit a duty-cycle pattern: each char 'S' or 'L' becomes one
   mark+gap symbol (S = long mark/short gap, L = short mark/long gap).
   The frame is repeated `repeats` times with a ~7ms inter-frame space,
   like a remote re-sending while the button is held (<=0 means 5).
   t_us = {s_mark, s_gap, l_mark, l_gap} in microseconds, or NULL for
   the fan-remote defaults 1240/440/400/1240. */
void ir_ctrl_send_pattern(const char *pattern, const uint32_t t_us[4],
                          int repeats);

/* Same as ir_ctrl_send_pattern but generates the carrier in software
   (38kHz encoded directly into the symbol stream - no RMT carrier hw). */
void ir_ctrl_send_pattern_pwm(const char *pattern, const uint32_t t_us[4],
                              int repeats);

/* Loopback self-test: TX one frame while RX (VS1838B) listens and
   print the decoded marks/gaps. Aim the TX LED at the receiver. */
void ir_ctrl_loopback(const char *pattern);

/* Passive capture: print every mark/gap the VS1838B decodes (~8s window).
   Used to dump the real remote's frame for comparison. */
void ir_ctrl_capture(void);

/* Start a one-shot background learn for the ON or OFF code. */
void ir_ctrl_learn(bool on);

/* True if a learn is currently in progress. */
bool ir_ctrl_is_learning(void);
