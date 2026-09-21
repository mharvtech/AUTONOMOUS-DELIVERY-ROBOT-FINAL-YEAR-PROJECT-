#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// Calibration that survives a power cycle. Without this the IR polarity, the
// line thresholds and both white balances would have to be re-taught on every
// boot - exactly the chore that gets skipped before a demo.
esp_err_t settings_init(void);

// Each loader returns false when nothing is stored yet, leaving the caller's
// defaults in place.
bool settings_load_ir_line_level(int *line_level);
bool settings_load_line_window(uint16_t *on_line, uint16_t *on_floor);
bool settings_load_white_balance(const char *which, uint16_t *r, uint16_t *g, uint16_t *b);

esp_err_t settings_save_ir_line_level(int line_level);
esp_err_t settings_save_line_window(uint16_t on_line, uint16_t on_floor);
// `which` is a short tag naming the sensor, e.g. "f" (front) or "b" (bottom).
esp_err_t settings_save_white_balance(const char *which, uint16_t r, uint16_t g, uint16_t b);

// Wipes stored calibration so the next boot starts from the compiled defaults.
esp_err_t settings_forget(void);

#ifdef __cplusplus
}
#endif
