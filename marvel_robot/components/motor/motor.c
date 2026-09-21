#include "motor.h"
#include <stdlib.h>
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"

static const char *TAG = "motor";

// ============================================================
//                      L298N pin map
// ============================================================
// One L298N drives both sides; the two motors on each side are wired in
// parallel onto that side's output pair:
//
//   OUT1/OUT2 (ENA, IN1/IN2) -> front-left  + rear-left  motors
//   OUT3/OUT4 (ENB, IN3/IN4) -> front-right + rear-right motors
//
// CURRENT WARNING: two motors in parallel draw double the current through one
// L298N channel. It is rated ~2A per channel and drops ~2V internally, so a
// stalled pair can trip its thermal shutdown - the robot goes dead for a few
// seconds, then comes back. That is a hardware fix (a second driver, or a
// TB6612/BTS7960), not a software one.
//
// NOTE: IN3 is GPIO12, a strapping pin (MTDI). If it is pulled HIGH at reset
// the ESP32 selects a 1.8V flash voltage and will not boot. If the board stops
// booting once the motor driver is connected, move IN3 to GPIO5 or GPIO17.
#define IN1_PIN 14
#define IN2_PIN 27
#define IN3_PIN 12
#define IN4_PIN 4
#define ENA_PIN 13
#define ENB_PIN 15

// ============================================================
//                    Orientation flags
// ============================================================
// These absorb how the motors are physically wired so that everywhere else a
// positive speed simply means "forward".
//
// How to check: prop the robot so the wheels spin free, switch to manual mode,
// press Forward. Every wheel should turn as if driving toward the ultrasonic
// sensor.
//   - all four spin backwards   -> flip BOTH invert flags
//   - the robot spins in place  -> flip ONE invert flag
//   - Left and Right are swapped-> set MOTOR_SWAP_SIDES to 1
#define MOTOR_LEFT_INVERT   1
#define MOTOR_RIGHT_INVERT  1
#define MOTOR_SWAP_SIDES    0

// Below roughly this duty a loaded gearmotor buzzes but does not turn, and one
// side starting to move before the other reads as a random veer. Any non-zero
// request is lifted to at least this. Set to 0 to disable.
#define MOTOR_MIN_DUTY 70

#define PWM_FREQ_HZ   5000
#define PWM_RES_BITS  LEDC_TIMER_8_BIT
#define LEDC_MODE     LEDC_LOW_SPEED_MODE
#define ENA_CHANNEL   LEDC_CHANNEL_0
#define ENB_CHANNEL   LEDC_CHANNEL_1

void motor_init(void)
{
    gpio_config_t dir_conf = {
        .pin_bit_mask = (1ULL << IN1_PIN) | (1ULL << IN2_PIN) |
                        (1ULL << IN3_PIN) | (1ULL << IN4_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&dir_conf);

    ledc_timer_config_t timer_conf = {
        .speed_mode = LEDC_MODE,
        .duty_resolution = PWM_RES_BITS,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = PWM_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&timer_conf);

    ledc_channel_config_t ena_conf = {
        .gpio_num = ENA_PIN,
        .speed_mode = LEDC_MODE,
        .channel = ENA_CHANNEL,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0,
    };
    ledc_channel_config(&ena_conf);

    ledc_channel_config_t enb_conf = ena_conf;
    enb_conf.gpio_num = ENB_PIN;
    enb_conf.channel = ENB_CHANNEL;
    ledc_channel_config(&enb_conf);

    motor_stop();
    ESP_LOGI(TAG, "Motor driver ready (L298N, one channel per side).");
}

static void set_side(int speed, int in_pin_fwd, int in_pin_rev, ledc_channel_t channel)
{
    if (speed > 255) speed = 255;
    if (speed < -255) speed = -255;

    int magnitude = abs(speed);
    if (magnitude > 0 && magnitude < MOTOR_MIN_DUTY) {
        magnitude = MOTOR_MIN_DUTY;
    }

    gpio_set_level(in_pin_fwd, speed >= 0 ? 1 : 0);
    gpio_set_level(in_pin_rev, speed >= 0 ? 0 : 1);

    ledc_set_duty(LEDC_MODE, channel, magnitude);
    ledc_update_duty(LEDC_MODE, channel);
}

void motor_set(int left_speed, int right_speed)
{
#if MOTOR_SWAP_SIDES
    int tmp = left_speed;
    left_speed = right_speed;
    right_speed = tmp;
#endif
#if MOTOR_LEFT_INVERT
    left_speed = -left_speed;
#endif
#if MOTOR_RIGHT_INVERT
    right_speed = -right_speed;
#endif

    set_side(left_speed, IN1_PIN, IN2_PIN, ENA_CHANNEL);
    set_side(right_speed, IN3_PIN, IN4_PIN, ENB_CHANNEL);
}

void motor_stop(void)
{
    motor_set(0, 0);
}

void motor_brake(void)
{
    // Active braking: hold both inputs on each side HIGH at full duty. This
    // shorts each motor's terminals through the H-bridge, resisting rotation
    // electrically - a much faster stop than cutting power and coasting.
    // Braking is symmetric, so the orientation flags do not apply.
    gpio_set_level(IN1_PIN, 1);
    gpio_set_level(IN2_PIN, 1);
    gpio_set_level(IN3_PIN, 1);
    gpio_set_level(IN4_PIN, 1);

    ledc_set_duty(LEDC_MODE, ENA_CHANNEL, 255);
    ledc_update_duty(LEDC_MODE, ENA_CHANNEL);
    ledc_set_duty(LEDC_MODE, ENB_CHANNEL, 255);
    ledc_update_duty(LEDC_MODE, ENB_CHANNEL);
}
