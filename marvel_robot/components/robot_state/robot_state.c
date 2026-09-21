#include "robot_state.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_timer.h"

static SemaphoreHandle_t s_mutex;

static robot_mode_t s_mode = ROBOT_MODE_AUTONOMOUS;
static bool         s_mission_running = false;
static color_id_t   s_target_color = COLOR_NONE;

static manual_direction_t s_manual_dir = MANUAL_STOP;
static int     s_manual_pwm = 0;
static int64_t s_manual_cmd_time_us = 0;

static robot_telemetry_t s_telemetry;
static bool s_brake_requested = false;

static calibration_request_t s_cal_request = CAL_NONE;
static bool s_cal_ok = false;
static char s_cal_message[CAL_MESSAGE_LEN] = "";

#define LOCK()   xSemaphoreTake(s_mutex, portMAX_DELAY)
#define UNLOCK() xSemaphoreGive(s_mutex)

void robot_state_init(void)
{
    s_mutex = xSemaphoreCreateMutex();
    s_manual_cmd_time_us = esp_timer_get_time();

    memset(&s_telemetry, 0, sizeof(s_telemetry));
    strncpy(s_telemetry.state, "idle", sizeof(s_telemetry.state) - 1);
    s_telemetry.distance_cm = -1.0f;
    s_telemetry.target = COLOR_NONE;
    s_telemetry.front.color = COLOR_NONE;
}

void robot_state_set_mode(robot_mode_t mode)
{
    LOCK();
    s_mode = mode;
    if (mode == ROBOT_MODE_MANUAL) {
        s_mission_running = false;   // switching to manual cancels any mission
    }
    UNLOCK();
}

robot_mode_t robot_state_get_mode(void)
{
    LOCK();
    robot_mode_t m = s_mode;
    UNLOCK();
    return m;
}

void robot_state_start_mission(color_id_t target_color)
{
    LOCK();
    s_target_color = target_color;
    s_mission_running = true;
    s_mode = ROBOT_MODE_AUTONOMOUS;
    s_telemetry.arrived = false;
    s_telemetry.arrived_via[0] = '\0';
    UNLOCK();
}

void robot_state_stop_mission(void)
{
    LOCK();
    s_mission_running = false;
    UNLOCK();
}

bool robot_state_is_mission_running(void)
{
    LOCK();
    bool r = s_mission_running;
    UNLOCK();
    return r;
}

color_id_t robot_state_get_target_color(void)
{
    LOCK();
    color_id_t c = s_target_color;
    UNLOCK();
    return c;
}

void robot_state_set_manual_command(manual_direction_t dir, int pwm)
{
    LOCK();
    s_manual_dir = dir;
    s_manual_pwm = pwm;
    s_manual_cmd_time_us = esp_timer_get_time();
    UNLOCK();
}

void robot_state_get_manual_command(manual_direction_t *dir, int *pwm, int64_t *age_us)
{
    LOCK();
    if (dir)    *dir = s_manual_dir;
    if (pwm)    *pwm = s_manual_pwm;
    if (age_us) *age_us = esp_timer_get_time() - s_manual_cmd_time_us;
    UNLOCK();
}

void robot_state_set_telemetry(const robot_telemetry_t *telemetry)
{
    if (!telemetry) return;
    LOCK();
    s_telemetry = *telemetry;
    s_telemetry.state[sizeof(s_telemetry.state) - 1] = '\0';
    s_telemetry.arrived_via[sizeof(s_telemetry.arrived_via) - 1] = '\0';
    UNLOCK();
}

void robot_state_get_telemetry(robot_telemetry_t *out)
{
    if (!out) return;
    LOCK();
    *out = s_telemetry;
    UNLOCK();
}

void robot_state_estop(void)
{
    LOCK();
    s_mission_running = false;
    s_manual_dir = MANUAL_STOP;
    s_manual_pwm = 0;
    s_manual_cmd_time_us = esp_timer_get_time();
    s_brake_requested = true;
    UNLOCK();
}

bool robot_state_take_brake_request(void)
{
    LOCK();
    bool requested = s_brake_requested;
    s_brake_requested = false;
    UNLOCK();
    return requested;
}

void robot_state_request_calibration(calibration_request_t request)
{
    LOCK();
    s_cal_request = request;
    UNLOCK();
}

calibration_request_t robot_state_take_calibration_request(void)
{
    LOCK();
    calibration_request_t r = s_cal_request;
    s_cal_request = CAL_NONE;
    UNLOCK();
    return r;
}

void robot_state_set_calibration_result(bool ok, const char *message)
{
    LOCK();
    s_cal_ok = ok;
    strncpy(s_cal_message, message ? message : "", sizeof(s_cal_message) - 1);
    s_cal_message[sizeof(s_cal_message) - 1] = '\0';
    UNLOCK();
}

void robot_state_get_calibration_result(bool *ok, char *message_out, size_t len)
{
    LOCK();
    if (ok) *ok = s_cal_ok;
    if (message_out && len > 0) {
        strncpy(message_out, s_cal_message, len - 1);
        message_out[len - 1] = '\0';
    }
    UNLOCK();
}

manual_direction_t manual_direction_from_string(const char *s)
{
    if (!s) return MANUAL_STOP;
    if (strcmp(s, "forward") == 0)  return MANUAL_FORWARD;
    if (strcmp(s, "backward") == 0) return MANUAL_BACKWARD;
    if (strcmp(s, "left") == 0)     return MANUAL_LEFT;
    if (strcmp(s, "right") == 0)    return MANUAL_RIGHT;
    if (strcmp(s, "spin_cw") == 0)  return MANUAL_SPIN_CW;
    if (strcmp(s, "spin_ccw") == 0) return MANUAL_SPIN_CCW;
    return MANUAL_STOP;
}

const char *manual_direction_to_string(manual_direction_t d)
{
    switch (d) {
        case MANUAL_FORWARD:  return "forward";
        case MANUAL_BACKWARD: return "backward";
        case MANUAL_LEFT:     return "left";
        case MANUAL_RIGHT:    return "right";
        case MANUAL_SPIN_CW:  return "spin_cw";
        case MANUAL_SPIN_CCW: return "spin_ccw";
        default:              return "stop";
    }
}

color_id_t color_from_string(const char *s)
{
    if (!s) return COLOR_NONE;
    if (strcmp(s, "red") == 0)   return COLOR_RED;
    if (strcmp(s, "green") == 0) return COLOR_GREEN;
    if (strcmp(s, "blue") == 0)  return COLOR_BLUE;
    return COLOR_NONE;   // "black" is deliberately absent: it is the guide
                         // line, never a destination
}

const char *color_to_lower_string(color_id_t c)
{
    switch (c) {
        case COLOR_RED:   return "red";
        case COLOR_GREEN: return "green";
        case COLOR_BLUE:  return "blue";
        case COLOR_BLACK: return "black";
        default:          return "none";
    }
}
