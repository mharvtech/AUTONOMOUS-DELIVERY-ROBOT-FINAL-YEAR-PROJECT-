#include <string.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "motor.h"
#include "ir_ultrasonic.h"
#include "color_sensor.h"
#include "robot_state.h"
#include "settings.h"
#include "web_server.h"

static const char *TAG = "delivery_robot";

// ============================================================
//                      Sensor layout
// ============================================================
//
//              [ TCS34725 ]      forward facing. Reads the coloured plate
//                   ^            outside each room. It takes NO part in line
//                   |            following - a colour reading needs ~24ms to
//                                integrate, far too slow to steer on.
//
//        IR_L  o  ================  o  IR_R
//                   black tape
//
// The two IR modules straddle the tape: centred, BOTH see bare floor. One of
// them going black means the tape has drifted under that side of the robot.

#define FRONT_I2C_PORT  0
#define FRONT_SDA_PIN   21
#define FRONT_SCL_PIN   22
#define COLOR_POLL_MS   40

// ============================================================
//                    Line following
// ============================================================
// Two straddling sensors give three readings - tape left, tape right, and
// "neither". The catch is that "neither" means both "you are centred" and
// "you have left the track entirely", and no amount of code separates those.
// What CAN be recovered is the size of the error, from how long a sensor stays
// black, and that is what the phases below are built on:
//
//   FORWARD  both sensors clear. Drive straight at cruise speed.
//   NUDGE    a sensor just went black. Small drift on a straight, most likely,
//            so ease that side and keep driving forward. If it clears quickly
//            that was all it was.
//   CORNER   the same sensor has been black for CORNER_AFTER_MS. A nudge would
//            have fixed a drift by now, so this is a real 90-degree corner.
//            Commit to a pivot on the spot.
//   RECOVER  the sensor cleared after a corner. Drive forward SLOWLY for a
//            moment instead of jumping back to cruise. If the sensor trips
//            again during this window the pivot ended early, and the robot
//            goes straight back to CORNER without waiting out NUDGE again.
//            This is what turns the classic corner stutter into two or three
//            decreasing pivots.
//   LOST     pivoted for PIVOT_TIMEOUT_MS without finding the tape. Stop.
//
// Losing the line on a STRAIGHT cannot be detected with two straddling
// sensors - it looks exactly like being centred. The ultrasonic backstops it
// by stopping the robot before a wall.

#define LOOP_PERIOD_MS 5

#define CRUISE_SPEED   165   // both sensors clear
#define NUDGE_SPEED    135   // forward speed while easing back onto the line
#define NUDGE_TRIM      95   // inside wheels: NUDGE_SPEED - NUDGE_TRIM
#define PIVOT_SPEED    150   // rotating on the spot through a corner
#define RECOVER_SPEED  120   // creeping out of a corner, ready to re-commit
#define JUNCTION_SPEED 120   // crossing a line that meets this one at 90 deg

// A drift on a straight clears in a few tens of milliseconds once the wheels
// respond. Still black after this and it is a corner, not a drift.
#define CORNER_AFTER_MS  120

// How long to creep after a corner before trusting cruise speed again.
#define RECOVER_MS       260

// A 90-degree pivot takes well under a second. Longer than this means the tape
// was never found, so stop rather than spin in the middle of the room.
#define PIVOT_TIMEOUT_MS 2500

// ---------------- Obstacle avoidance ----------------
#define OBSTACLE_DISTANCE_CM  15
#define TURN_SPEED           150
#define TURN_90_DURATION_MS  500   // calibrate for your chassis
#define TURN_SLICE_MS         20

// ---------------- Mission ----------------
#define COLOR_STABLE_READS 3

// ---------------- Manual mode safety ----------------
#define MANUAL_CMD_TIMEOUT_US 1500000   // 1.5s of silence -> stop

// ---------------- Loop timing, resolved at boot ----------------
// pdMS_TO_TICKS() truncates. At the ESP-IDF default CONFIG_FREERTOS_HZ=100 one
// tick is 10ms, so pdMS_TO_TICKS(5) is ZERO - and vTaskDelay(0) does not block,
// it only yields to tasks of equal or higher priority. The loop would spin, the
// CPU0 idle task would never run, and the task watchdog would fire seconds
// later. Clamp to at least one tick.
static TickType_t s_loop_delay_ticks = 1;
static int s_loop_ms = 10;

static color_sensor_handle_t s_front_sensor = NULL;
static color_reading_t s_front_reading;
static int64_t s_last_color_read_us = 0;

// ---------------- Line-follower state ----------------
typedef enum {
    LF_FORWARD,
    LF_NUDGE,
    LF_CORNER,
    LF_RECOVER,
    LF_LOST
} lf_phase_t;

static lf_phase_t s_phase = LF_FORWARD;
static int        s_dir = 0;               // -1 tape is left, +1 tape is right
static int64_t    s_black_since_us = 0;    // when the current sensor went black
static int64_t    s_pivot_started_us = 0;
static int64_t    s_recover_started_us = 0;

static color_id_t s_stable_color = COLOR_NONE;
static int        s_stable_count = 0;

static void reset_line_follower(void)
{
    s_phase = LF_FORWARD;
    s_dir = 0;
    s_black_since_us = 0;
    s_pivot_started_us = 0;
    s_recover_started_us = 0;
}

// ============================================================
//                     Colour sensor
// ============================================================
static bool refresh_color_sensor(void)
{
    int64_t now = esp_timer_get_time();
    if (s_last_color_read_us != 0 &&
        (now - s_last_color_read_us) < (COLOR_POLL_MS * 1000)) {
        return false;
    }
    s_last_color_read_us = now;
    s_front_reading = color_sensor_read(s_front_sensor);
    return true;
}

// A dark reading from the forward sensor is an unlit wall, not a room.
static color_id_t destination_color(void)
{
    if (!s_front_reading.valid) return COLOR_NONE;
    if (s_front_reading.color == COLOR_BLACK) return COLOR_NONE;
    return s_front_reading.color;
}

// ============================================================
//                       Manoeuvres
// ============================================================
static bool turn_90_degrees(void)
{
    motor_set(TURN_SPEED, -TURN_SPEED);
    for (int elapsed = 0; elapsed < TURN_90_DURATION_MS; elapsed += TURN_SLICE_MS) {
        vTaskDelay(pdMS_TO_TICKS(TURN_SLICE_MS));
        if (robot_state_get_mode() != ROBOT_MODE_AUTONOMOUS ||
            !robot_state_is_mission_running()) {
            motor_stop();
            ESP_LOGW(TAG, "Avoidance turn aborted.");
            return false;
        }
    }
    motor_stop();
    reset_line_follower();
    return true;
}

// ============================================================
//                      Calibration
// ============================================================
static void service_calibration_request(void)
{
    calibration_request_t req = robot_state_take_calibration_request();
    if (req == CAL_NONE) return;

    motor_stop();   // never sample while the wheels are turning

    char msg[CAL_MESSAGE_LEN];
    bool ok = false;
    uint16_t r, g, b;

    switch (req) {
        case CAL_IR_FLOOR:
            // Centred on the tape, both IR modules are over bare floor by
            // definition of the straddling layout, so whatever level they read
            // now is "floor" and the opposite is "black". This removes the
            // guess about whether the modules are active-high or active-low.
            ok = (ir_calibrate_floor() == ESP_OK);
            if (ok) {
                settings_save_ir_line_level(ir_get_line_level());
                snprintf(msg, sizeof(msg), "IR polarity learned: black reads %s",
                         ir_get_line_level() ? "HIGH" : "LOW");
            } else {
                snprintf(msg, sizeof(msg), "IR calibration failed - centre the robot "
                                           "so both IR sensors sit on bare floor");
            }
            break;

        case CAL_WHITE:
            ok = (color_sensor_calibrate_white(s_front_sensor) == ESP_OK);
            if (ok) {
                color_sensor_get_white_balance(s_front_sensor, &r, &g, &b);
                settings_save_white_balance("f", r, g, b);
                snprintf(msg, sizeof(msg), "White balance learned: R%u G%u B%u", r, g, b);
            } else {
                snprintf(msg, sizeof(msg), "White balance failed - hold white paper "
                                           "in front of the sensor");
            }
            break;

        case CAL_FORGET:
            settings_forget();
            ok = true;
            snprintf(msg, sizeof(msg), "Stored calibration erased - reboot for defaults");
            break;

        default:
            return;
    }

    robot_state_set_calibration_result(ok, msg);
    ESP_LOGI(TAG, "Calibration: %s", msg);
    s_last_color_read_us = 0;
}

// ============================================================
//                       Manual mode
// ============================================================
static void apply_manual_command(manual_direction_t dir, int pwm,
                                 int *out_left, int *out_right)
{
    int left = 0, right = 0;
    switch (dir) {
        case MANUAL_FORWARD:  left =  pwm;     right =  pwm;     break;
        case MANUAL_BACKWARD: left = -pwm;     right = -pwm;     break;
        case MANUAL_LEFT:     left =  pwm / 2; right =  pwm;     break;
        case MANUAL_RIGHT:    left =  pwm;     right =  pwm / 2; break;
        case MANUAL_SPIN_CW:  left =  pwm;     right = -pwm;     break;
        case MANUAL_SPIN_CCW: left = -pwm;     right =  pwm;     break;
        default:              left = 0;        right = 0;        break;
    }
    motor_set(left, right);
    *out_left = left;
    *out_right = right;
}

static void run_manual_mode(robot_telemetry_t *t)
{
    manual_direction_t dir;
    int pwm;
    int64_t age_us;
    robot_state_get_manual_command(&dir, &pwm, &age_us);

    if (dir == MANUAL_STOP) {
        motor_stop();
        t->left_speed = 0;
        t->right_speed = 0;
        strncpy(t->state, "manual_idle", sizeof(t->state) - 1);
    } else if (age_us > MANUAL_CMD_TIMEOUT_US) {
        // Link lost mid-drive. Latch the stop so a stale heartbeat arriving
        // late cannot restart the motors on its own.
        motor_stop();
        robot_state_set_manual_command(MANUAL_STOP, 0);
        t->left_speed = 0;
        t->right_speed = 0;
        strncpy(t->state, "link_lost", sizeof(t->state) - 1);
        ESP_LOGW(TAG, "Manual command timed out (%lldms) - motors stopped.",
                 age_us / 1000);
    } else {
        apply_manual_command(dir, pwm, &t->left_speed, &t->right_speed);
        strncpy(t->state, manual_direction_to_string(dir), sizeof(t->state) - 1);
    }
}

// ============================================================
//                    The line follower
// ============================================================
static void follow_line(const ir_state_t *ir, robot_telemetry_t *t,
                        int *out_left, int *out_right, const char **out_state)
{
    int64_t now = esp_timer_get_time();

    // ---- Both sensors black: a line crossing at right angles ----
    // Creep across rather than stopping, or the robot strands itself on every
    // doorway bar.
    if (ir->left && ir->right) {
        reset_line_follower();
        *out_left = JUNCTION_SPEED;
        *out_right = JUNCTION_SPEED;
        *out_state = "junction";
        t->turn_dir = 0;
        t->black_ms = 0;
        return;
    }

    int black = ir->left ? -1 : (ir->right ? 1 : 0);

    switch (s_phase) {
        case LF_LOST:
            break;   // sticky until the follower is reset

        case LF_FORWARD:
            if (black != 0) {
                s_phase = LF_NUDGE;
                s_dir = black;
                s_black_since_us = now;
            }
            break;

        case LF_NUDGE:
            if (black == 0) {
                s_phase = LF_FORWARD;      // it really was just a drift
                s_dir = 0;
            } else if (black != s_dir) {
                s_dir = black;             // the other edge clipped instead
                s_black_since_us = now;
            } else if ((now - s_black_since_us) > (int64_t)CORNER_AFTER_MS * 1000) {
                // Easing the wheels has not cleared it, so this is a corner.
                s_phase = LF_CORNER;
                s_pivot_started_us = now;
                ESP_LOGI(TAG, "Corner detected to the %s - pivoting.",
                         s_dir < 0 ? "left" : "right");
            }
            break;

        case LF_CORNER:
            if (black != 0 && black != s_dir) {
                s_dir = black;             // overshot: pivot back the other way
                s_pivot_started_us = now;
            } else if (black == 0) {
                s_phase = LF_RECOVER;
                s_recover_started_us = now;
            } else if ((now - s_pivot_started_us) > (int64_t)PIVOT_TIMEOUT_MS * 1000) {
                s_phase = LF_LOST;
                ESP_LOGW(TAG, "Pivoted %dms without finding the tape - stopping.",
                         PIVOT_TIMEOUT_MS);
            }
            break;

        case LF_RECOVER:
            if (black != 0) {
                // The pivot ended early. Go straight back to pivoting rather
                // than waiting out NUDGE again - we already know we are in a
                // corner, and each pass through here is a smaller correction.
                s_phase = LF_CORNER;
                s_dir = black;
                s_pivot_started_us = now;
            } else if ((now - s_recover_started_us) > (int64_t)RECOVER_MS * 1000) {
                s_phase = LF_FORWARD;
                s_dir = 0;
            }
            break;
    }

    t->turn_dir = (s_phase == LF_NUDGE || s_phase == LF_CORNER) ? s_dir : 0;
    t->black_ms = (black != 0 && s_black_since_us != 0)
                  ? (int)((now - s_black_since_us) / 1000) : 0;

    switch (s_phase) {
        case LF_FORWARD:
            *out_left = CRUISE_SPEED;
            *out_right = CRUISE_SPEED;
            *out_state = "forward";
            break;

        case LF_NUDGE:
            // Ease the wheels on the side the tape drifted to, still driving
            // forward. A gentle arc, because most trips are small drifts.
            if (s_dir < 0) {
                *out_left  = NUDGE_SPEED - NUDGE_TRIM;
                *out_right = NUDGE_SPEED;
                *out_state = "nudge_left";
            } else {
                *out_left  = NUDGE_SPEED;
                *out_right = NUDGE_SPEED - NUDGE_TRIM;
                *out_state = "nudge_right";
            }
            break;

        case LF_CORNER:
            if (s_dir < 0) {
                *out_left  = -PIVOT_SPEED;
                *out_right =  PIVOT_SPEED;
                *out_state = "corner_left";
            } else {
                *out_left  =  PIVOT_SPEED;
                *out_right = -PIVOT_SPEED;
                *out_state = "corner_right";
            }
            break;

        case LF_RECOVER:
            *out_left = RECOVER_SPEED;
            *out_right = RECOVER_SPEED;
            *out_state = "recover";
            break;

        case LF_LOST:
        default:
            *out_left = 0;
            *out_right = 0;
            *out_state = "line_lost";
            break;
    }
}

// ============================================================
//                     Autonomous mode
// ============================================================
static bool check_arrival(color_id_t target, robot_telemetry_t *t)
{
    color_id_t seen = destination_color();

    if (seen != COLOR_NONE && seen == target) {
        s_stable_count = (seen == s_stable_color) ? s_stable_count + 1 : 1;
        s_stable_color = seen;
    } else {
        s_stable_count = 0;
        s_stable_color = COLOR_NONE;
    }

    if (s_stable_count < COLOR_STABLE_READS) return false;

    motor_brake();
    vTaskDelay(pdMS_TO_TICKS(150));
    motor_stop();
    robot_state_stop_mission();
    s_stable_count = 0;
    s_stable_color = COLOR_NONE;

    t->arrived = true;
    strncpy(t->arrived_via, "front", sizeof(t->arrived_via) - 1);
    t->left_speed = 0;
    t->right_speed = 0;
    t->turn_dir = 0;
    strncpy(t->state, "arrived", sizeof(t->state) - 1);
    ESP_LOGI(TAG, "Target colour %s confirmed - mission complete.",
             color_to_lower_string(target));
    return true;
}

static void run_autonomous_mode(robot_telemetry_t *t, bool fresh_color)
{
    if (!robot_state_is_mission_running()) {
        motor_stop();
        reset_line_follower();
        t->left_speed = 0;
        t->right_speed = 0;
        t->turn_dir = 0;
        t->black_ms = 0;
        if (!t->arrived) {
            strncpy(t->state, "idle", sizeof(t->state) - 1);
        }
        return;
    }

    t->arrived = false;
    t->arrived_via[0] = '\0';
    color_id_t target = robot_state_get_target_color();

    // ---- 1. Obstacle avoidance - highest priority ----
    if (t->distance_cm >= 0.0f && t->distance_cm <= OBSTACLE_DISTANCE_CM) {
        strncpy(t->state, "obstacle", sizeof(t->state) - 1);
        t->left_speed = 0;
        t->right_speed = 0;
        robot_state_set_telemetry(t);

        ESP_LOGI(TAG, "Obstacle at ~%.1f cm - braking and turning.", t->distance_cm);
        motor_brake();
        vTaskDelay(pdMS_TO_TICKS(120));
        turn_90_degrees();
        return;
    }

    // ---- 2. Arrival, only on fresh colour samples ----
    if (fresh_color && check_arrival(target, t)) {
        return;
    }

    // ---- 3. Line following ----
    ir_state_t ir = ir_read();
    t->ir_left = ir.left;
    t->ir_right = ir.right;

    int left = 0, right = 0;
    const char *behavior = "forward";
    follow_line(&ir, t, &left, &right, &behavior);

    motor_set(left, right);
    t->left_speed = left;
    t->right_speed = right;
    strncpy(t->state, behavior, sizeof(t->state) - 1);
}

// ============================================================
//                          Setup
// ============================================================
static void init_loop_timing(void)
{
    s_loop_delay_ticks = pdMS_TO_TICKS(LOOP_PERIOD_MS);
    if (s_loop_delay_ticks < 1) {
        s_loop_delay_ticks = 1;   // never vTaskDelay(0): it starves the idle task
    }
    s_loop_ms = (int)(s_loop_delay_ticks * portTICK_PERIOD_MS);

    ESP_LOGI(TAG, "Control loop: %dms per pass (%d tick(s) of %dms).",
             s_loop_ms, (int)s_loop_delay_ticks, (int)portTICK_PERIOD_MS);
    if (s_loop_ms > LOOP_PERIOD_MS) {
        ESP_LOGW(TAG, "Wanted a %dms loop but the FreeRTOS tick is %dms. Set "
                      "CONFIG_FREERTOS_HZ=1000 (sdkconfig.defaults ships with it) "
                      "for a faster reaction at corners.",
                 LOOP_PERIOD_MS, (int)portTICK_PERIOD_MS);
    }
}

void app_main(void)
{
    init_loop_timing();
    motor_init();
    ir_ultrasonic_init();
    robot_state_init();
    settings_init();

    color_sensor_config_t front_cfg = {
        .i2c_port = FRONT_I2C_PORT, .sda_gpio = FRONT_SDA_PIN, .scl_gpio = FRONT_SCL_PIN,
        .atime = COLOR_ATIME_101MS, .gain = COLOR_GAIN_16X, .name = "front",
    };
    if (color_sensor_init(&front_cfg, &s_front_sensor) != ESP_OK) {
        ESP_LOGE(TAG, "Front colour sensor init failed. Line following and manual "
                      "driving still work; a mission cannot complete.");
        s_front_sensor = NULL;
    }

    int stored_level;
    if (settings_load_ir_line_level(&stored_level)) {
        ir_set_line_level(stored_level);
    } else {
        ESP_LOGW(TAG, "No stored IR polarity. Centre the robot on the tape and run "
                      "\"Learn IR polarity\" from the control panel.");
    }

    uint16_t wr, wg, wb;
    if (settings_load_white_balance("f", &wr, &wg, &wb)) {
        color_sensor_set_white_balance(s_front_sensor, wr, wg, wb);
    } else {
        ESP_LOGW(TAG, "No stored white balance. Hold white paper in front of the "
                      "colour sensor and run \"Learn white balance\".");
    }

    if (web_server_start() != ESP_OK) {
        ESP_LOGE(TAG, "Web server failed to start - no remote control this session.");
    }

    ESP_LOGI(TAG, "Robot ready.");

    robot_telemetry_t t;
    memset(&t, 0, sizeof(t));
    strncpy(t.state, "idle", sizeof(t.state) - 1);
    t.distance_cm = -1.0f;

    while (1) {
        service_calibration_request();

        bool fresh_color = refresh_color_sensor();

        if (robot_state_take_brake_request()) {
            motor_brake();
            vTaskDelay(pdMS_TO_TICKS(150));
            motor_stop();
            reset_line_follower();
            strncpy(t.state, "estop", sizeof(t.state) - 1);
            t.left_speed = 0;
            t.right_speed = 0;
            t.turn_dir = 0;
            t.arrived = false;
            robot_state_set_telemetry(&t);
            continue;
        }

        t.front = s_front_reading;
        t.distance_cm = ultrasonic_get_distance_cm();
        t.target = robot_state_get_target_color();
        t.mission_running = robot_state_is_mission_running();
        t.ir_line_level = ir_get_line_level();
        color_sensor_get_white_balance(s_front_sensor, &t.wb_r, &t.wb_g, &t.wb_b);

        // Raw pin levels every pass, so a miswired or inverted IR module is
        // visible in the UI without a serial monitor.
        ir_state_t instantaneous;
        ir_read_raw(&t.ir_left_level, &t.ir_right_level, &instantaneous);

        if (robot_state_get_mode() == ROBOT_MODE_MANUAL) {
            ir_state_t ir = ir_read();
            t.ir_left = ir.left;
            t.ir_right = ir.right;
            t.turn_dir = 0;
            t.black_ms = 0;
            t.arrived = false;
            run_manual_mode(&t);
        } else {
            run_autonomous_mode(&t, fresh_color);
        }

        robot_state_set_telemetry(&t);
        vTaskDelay(s_loop_delay_ticks);
    }
}
