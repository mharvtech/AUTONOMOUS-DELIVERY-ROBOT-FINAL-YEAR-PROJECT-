#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// COLOR_BLACK is what the centre sensor reports over the guide line. The front
// sensor can also report it (an unlit wall), but main.c never treats black as a
// destination.
typedef enum {
    COLOR_NONE,
    COLOR_BLACK,
    COLOR_RED,
    COLOR_GREEN,
    COLOR_BLUE
} color_id_t;

// One handle per physical TCS34725. The address is fixed at 0x29 with no
// select pin, so each sensor needs its own I2C port - two sensors on separate
// GPIOs but the SAME bus would both answer at once.
typedef struct color_sensor_s *color_sensor_handle_t;

// Integration time (ATIME register). The centre sensor wants speed, because
// the robot is moving over the surface it is measuring; the front sensor wants
// accuracy.
#define COLOR_ATIME_24MS   0xF6
#define COLOR_ATIME_50MS   0xEB
#define COLOR_ATIME_101MS  0xD5
#define COLOR_ATIME_154MS  0xC0

// Gain (CONTROL register)
#define COLOR_GAIN_1X   0x00
#define COLOR_GAIN_4X   0x01
#define COLOR_GAIN_16X  0x02
#define COLOR_GAIN_60X  0x03

typedef struct {
    int     i2c_port;
    int     sda_gpio;
    int     scl_gpio;
    uint8_t atime;
    uint8_t gain;
    const char *name;
} color_sensor_config_t;

typedef struct {
    bool       valid;       // false if the sensor is absent or the read failed
    uint16_t   r, g, b, c;  // raw channel counts (c = clear/brightness)
    color_id_t color;
    bool       on_line;     // clear channel at or below the "line" threshold
    int        darkness;    // 0 (bright floor) .. 100 (fully dark), graded
} color_reading_t;

esp_err_t color_sensor_init(const color_sensor_config_t *cfg,
                            color_sensor_handle_t *out_handle);

// Raw channels, classification and the graded darkness. Safe with a NULL handle.
color_reading_t color_sensor_read(color_sensor_handle_t handle);

// ---------------- Calibration ----------------
// Call from the task that reads the sensors - the I2C buses are not mutex
// guarded. HTTP handlers raise a request flag and the control loop performs it.

// Clear-channel window.
//   clear_on_line  - at or below this the surface counts as the black line
//   clear_on_floor - at or above this it counts as bare floor
// Between them, darkness ramps 100 -> 0. Defaults: 300 / 1500.
void color_sensor_set_line_window(color_sensor_handle_t handle,
                                  uint16_t clear_on_line, uint16_t clear_on_floor);
void color_sensor_get_line_window(color_sensor_handle_t handle,
                                  uint16_t *clear_on_line, uint16_t *clear_on_floor);

// Sample whatever is under the sensor now and store it as one end of the
// window. Park over the tape, call _line; park over bare floor, call _floor.
esp_err_t color_sensor_calibrate_line(color_sensor_handle_t handle);
esp_err_t color_sensor_calibrate_floor(color_sensor_handle_t handle);

// The TCS34725's channels are not equally sensitive - green reads highest even
// against neutral white - so raw ratios drift toward green and blue markers get
// missed. Sample white paper and every later classification is normalised to it.
esp_err_t color_sensor_calibrate_white(color_sensor_handle_t handle);
void color_sensor_set_white_balance(color_sensor_handle_t handle,
                                    uint16_t r, uint16_t g, uint16_t b);
void color_sensor_get_white_balance(color_sensor_handle_t handle,
                                    uint16_t *r, uint16_t *g, uint16_t *b);

const char *color_name(color_id_t color);

#ifdef __cplusplus
}
#endif
