#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include "color_sensor.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    ROBOT_MODE_AUTONOMOUS,
    ROBOT_MODE_MANUAL
} robot_mode_t;

typedef enum {
    MANUAL_STOP,
    MANUAL_FORWARD,
    MANUAL_BACKWARD,
    MANUAL_LEFT,
    MANUAL_RIGHT,
    MANUAL_SPIN_CW,
    MANUAL_SPIN_CCW
} manual_direction_t;

// Longest calibration message any of these can produce, shared by the
// producer, the store and the HTTP reader so the three cannot drift apart.
#define CAL_MESSAGE_LEN 128

// Calibration is requested by an HTTP handler but performed by the control
// loop, because only that task may touch the I2C buses and the motors.
typedef enum {
    CAL_NONE = 0,
    CAL_IR_FLOOR,   // robot centred on the line: learn the IR polarity
    CAL_WHITE,      // front sensor on white paper
    CAL_FORGET      // erase stored calibration
} calibration_request_t;

// Everything the control loop publishes for the web UI.
typedef struct {
    char            state[24];      // "forward", "turn_left", "line_lost", ...
    float           distance_cm;    // -1 when nothing valid is in range

    color_reading_t front;          // forward sensor: room markers. It plays no
                                    // part in line following.

    bool            ir_left;        // debounced: this edge is over black
    bool            ir_right;
    int             ir_left_level;  // raw pin levels, for wiring diagnosis
    int             ir_right_level;
    int             ir_line_level;  // the level currently taken to mean "black"

    int             turn_dir;       // -1 turning left, 0 straight, +1 right
    int             black_ms;       // how long the tripped IR has been on black

    int             left_speed;
    int             right_speed;

    color_id_t      target;
    bool            mission_running;
    bool            arrived;
    char            arrived_via[8]; // "", "front", "bottom"

    uint16_t        wb_r, wb_g, wb_b;   // front sensor white reference
} robot_telemetry_t;

void robot_state_init(void);

// ---------------- Mode ----------------
void robot_state_set_mode(robot_mode_t mode);
robot_mode_t robot_state_get_mode(void);

// ---------------- Autonomous mission ----------------
void robot_state_start_mission(color_id_t target_color);
void robot_state_stop_mission(void);
bool robot_state_is_mission_running(void);
color_id_t robot_state_get_target_color(void);

// ---------------- Manual driving ----------------
void robot_state_set_manual_command(manual_direction_t dir, int pwm);
void robot_state_get_manual_command(manual_direction_t *dir, int *pwm, int64_t *age_us);

// ---------------- Telemetry ----------------
void robot_state_set_telemetry(const robot_telemetry_t *telemetry);
void robot_state_get_telemetry(robot_telemetry_t *out);

// ---------------- Emergency stop ----------------
void robot_state_estop(void);
bool robot_state_take_brake_request(void);   // true exactly once per e-stop

// ---------------- Calibration hand-off ----------------
void robot_state_request_calibration(calibration_request_t request);
calibration_request_t robot_state_take_calibration_request(void);
void robot_state_set_calibration_result(bool ok, const char *message);
void robot_state_get_calibration_result(bool *ok, char *message_out, size_t len);

// ---------------- String <-> enum helpers ----------------
manual_direction_t manual_direction_from_string(const char *s);
const char *manual_direction_to_string(manual_direction_t d);
color_id_t color_from_string(const char *s);      // "red"/"green"/"blue"
const char *color_to_lower_string(color_id_t c);  // -> "red"/"black"/.../"none"

#ifdef __cplusplus
}
#endif
