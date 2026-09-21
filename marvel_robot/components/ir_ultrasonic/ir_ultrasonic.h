#pragma once

#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// The two IR line sensors straddle the black line: when the robot is centred,
// BOTH sit on bare floor and neither is triggered. A sensor going true means
// the line has drifted under that side of the robot.
typedef struct {
    bool left;
    bool right;
} ir_state_t;

void ir_ultrasonic_init(void);

// Debounced read - a 3-sample majority per sensor. Call this exactly once per
// control-loop pass; each call shifts the filter window.
ir_state_t ir_read(void);

// Undebounced view, for the diagnostics panel: the actual pin levels and the
// instantaneous interpretation.
void ir_read_raw(int *left_level, int *right_level, ir_state_t *interpreted);

// ---------------- Polarity ----------------
// Some IR modules pull their output HIGH over black, others LOW; the indicator
// LED on the board does not reliably tell you which. Rather than hard-code a
// guess, the robot learns it: ir_calibrate_floor() is called with the robot
// centred on the line, where BOTH sensors are over bare floor by definition of
// the straddling geometry. Whatever level they both read then is the floor
// level, and the opposite means line.
esp_err_t ir_calibrate_floor(void);
void ir_set_line_level(int level);   // 0 or 1: the level that means "line"
int  ir_get_line_level(void);

// ---------------- Ultrasonic ----------------
// Median-filtered distance in cm, or -1.0f if nothing valid is in range. Safe
// to call every control-loop pass: the sensor is only pinged every 60ms (the
// datasheet minimum) and the cached value is returned in between.
float ultrasonic_get_distance_cm(void);

// One unfiltered, un-rate-limited ping. Blocks up to ~25ms. Bench use only.
float ultrasonic_ping_raw_cm(void);

#ifdef __cplusplus
}
#endif
