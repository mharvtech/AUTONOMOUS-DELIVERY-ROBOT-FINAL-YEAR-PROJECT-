#include "ir_ultrasonic.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "esp_log.h"

static const char *TAG = "ir_ultrasonic";

// ---------------- IR line sensor pins ----------------
#define IR_LEFT_PIN   32
#define IR_RIGHT_PIN  33

// The level that means "line". Overwritten by ir_calibrate_floor() and by the
// value restored from NVS at boot, so this is only the value used on a robot
// that has never been calibrated.
#define IR_DEFAULT_LINE_LEVEL 1

// A comparator sitting right on the edge of the line chatters. Three samples
// at the 5ms loop rate is a 15ms window - short enough not to lag the steering,
// long enough that one noisy sample cannot swing the wheels.
#define IR_FILTER_SAMPLES 3

// ---------------- HC-SR04 pins ----------------
#define TRIG_PIN 18
#define ECHO_PIN 19

// NOTE: HC-SR04 ECHO outputs 5V logic and ESP32 GPIOs are only 3.3V tolerant.
// Use a divider (1k + 2k) or a level shifter between ECHO and GPIO19.

// The datasheet asks for >=60ms between triggers. Ping faster and the tail of
// the previous burst is still bouncing around the room, so the next ECHO pulse
// can be a reflection of the LAST ping - which reads as a randomly short
// distance. Pings are rate limited here and the cached value returned between.
#define PING_INTERVAL_US 60000

// The HC-SR04 raises ECHO within ~500us of the trigger. If it has not gone high
// in 5ms the sensor is unpowered, miswired or dead - a different failure from
// "nothing in range", and it should not cost 30ms to discover.
#define ECHO_RISE_TIMEOUT_US 5000
#define ECHO_HIGH_TIMEOUT_US 25000   // ~4.3m round trip

#define DIST_MIN_CM   2.0f
#define DIST_MAX_CM 400.0f
#define MEDIAN_WINDOW 3

static int s_line_level = IR_DEFAULT_LINE_LEVEL;

static bool s_left_hist[IR_FILTER_SAMPLES];
static bool s_right_hist[IR_FILTER_SAMPLES];
static int  s_hist_idx = 0;

static int64_t s_last_ping_us = 0;
static float   s_filtered_cm  = -1.0f;
static float   s_history[MEDIAN_WINDOW] = { -1.0f, -1.0f, -1.0f };
static int     s_history_idx = 0;

void ir_ultrasonic_init(void)
{
    gpio_config_t ir_conf = {
        .pin_bit_mask = (1ULL << IR_LEFT_PIN) | (1ULL << IR_RIGHT_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&ir_conf);

    gpio_config_t trig_conf = {
        .pin_bit_mask = (1ULL << TRIG_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&trig_conf);

    gpio_config_t echo_conf = {
        .pin_bit_mask = (1ULL << ECHO_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&echo_conf);

    gpio_set_level(TRIG_PIN, 0);

    for (int i = 0; i < IR_FILTER_SAMPLES; i++) {
        s_left_hist[i] = false;
        s_right_hist[i] = false;
    }

    ESP_LOGI(TAG, "2 IR line sensors (GPIO%d/%d) + HC-SR04 ready.",
             IR_LEFT_PIN, IR_RIGHT_PIN);
}

void ir_set_line_level(int level)
{
    s_line_level = level ? 1 : 0;
    ESP_LOGI(TAG, "IR polarity: line reads %s.", s_line_level ? "HIGH" : "LOW");
}

int ir_get_line_level(void)
{
    return s_line_level;
}

void ir_read_raw(int *left_level, int *right_level, ir_state_t *interpreted)
{
    int l = gpio_get_level(IR_LEFT_PIN);
    int r = gpio_get_level(IR_RIGHT_PIN);
    if (left_level)  *left_level = l;
    if (right_level) *right_level = r;
    if (interpreted) {
        interpreted->left  = (l == s_line_level);
        interpreted->right = (r == s_line_level);
    }
}

ir_state_t ir_read(void)
{
    ir_state_t now;
    ir_read_raw(NULL, NULL, &now);

    s_left_hist[s_hist_idx] = now.left;
    s_right_hist[s_hist_idx] = now.right;
    s_hist_idx = (s_hist_idx + 1) % IR_FILTER_SAMPLES;

    int left_votes = 0, right_votes = 0;
    for (int i = 0; i < IR_FILTER_SAMPLES; i++) {
        if (s_left_hist[i]) left_votes++;
        if (s_right_hist[i]) right_votes++;
    }

    ir_state_t out;
    out.left  = left_votes  * 2 > IR_FILTER_SAMPLES;
    out.right = right_votes * 2 > IR_FILTER_SAMPLES;
    return out;
}

esp_err_t ir_calibrate_floor(void)
{
    // Sample both pins a few times so a single chattering read cannot set the
    // polarity for the whole robot.
    // ~5ms of sampling in total. Short enough to busy-wait without upsetting
    // the watchdog, long enough to ride out comparator chatter.
    int left_high = 0, right_high = 0;
    const int samples = 9;
    for (int i = 0; i < samples; i++) {
        left_high  += gpio_get_level(IR_LEFT_PIN);
        right_high += gpio_get_level(IR_RIGHT_PIN);
        esp_rom_delay_us(500);
    }

    int left_level  = (left_high  * 2 > samples) ? 1 : 0;
    int right_level = (right_high * 2 > samples) ? 1 : 0;

    if (left_level != right_level) {
        ESP_LOGW(TAG, "IR calibration: the two sensors disagree (L=%d R=%d). "
                      "The robot must be centred so BOTH sit on bare floor.",
                 left_level, right_level);
        return ESP_FAIL;
    }

    // Both are on floor right now, so the opposite level means line.
    ir_set_line_level(!left_level);
    return ESP_OK;
}

float ultrasonic_ping_raw_cm(void)
{
    // If ECHO is already high we are looking at the tail of a previous burst,
    // not a fresh measurement. Timing from here gives a truncated pulse and so
    // a falsely short distance - the classic phantom obstacle.
    if (gpio_get_level(ECHO_PIN) != 0) {
        return -1.0f;
    }

    gpio_set_level(TRIG_PIN, 0);
    esp_rom_delay_us(2);
    gpio_set_level(TRIG_PIN, 1);
    esp_rom_delay_us(10);
    gpio_set_level(TRIG_PIN, 0);

    int64_t wait_start = esp_timer_get_time();
    while (gpio_get_level(ECHO_PIN) == 0) {
        if (esp_timer_get_time() - wait_start > ECHO_RISE_TIMEOUT_US) {
            return -1.0f;  // sensor never responded - wiring or power
        }
    }

    int64_t echo_start = esp_timer_get_time();
    while (gpio_get_level(ECHO_PIN) == 1) {
        if (esp_timer_get_time() - echo_start > ECHO_HIGH_TIMEOUT_US) {
            return -1.0f;  // nothing came back in range
        }
    }
    int64_t echo_end = esp_timer_get_time();

    float cm = (float)(echo_end - echo_start) * 0.0343f / 2.0f;
    if (cm < DIST_MIN_CM || cm > DIST_MAX_CM) {
        return -1.0f;
    }
    return cm;
}

float ultrasonic_get_distance_cm(void)
{
    int64_t now = esp_timer_get_time();

    if (s_last_ping_us != 0 && (now - s_last_ping_us) < PING_INTERVAL_US) {
        return s_filtered_cm;
    }
    s_last_ping_us = now;

    s_history[s_history_idx] = ultrasonic_ping_raw_cm();
    s_history_idx = (s_history_idx + 1) % MEDIAN_WINDOW;

    // Median of the valid readings. A single glitched ping - very common on a
    // chassis that vibrates - can no longer trigger an avoidance turn on its
    // own; it takes two of the last three.
    float valid[MEDIAN_WINDOW];
    int n = 0;
    for (int i = 0; i < MEDIAN_WINDOW; i++) {
        if (s_history[i] >= 0.0f) valid[n++] = s_history[i];
    }
    if (n == 0) {
        s_filtered_cm = -1.0f;
        return s_filtered_cm;
    }
    for (int i = 1; i < n; i++) {          // insertion sort, n <= 3
        float key = valid[i];
        int j = i - 1;
        while (j >= 0 && valid[j] > key) { valid[j + 1] = valid[j]; j--; }
        valid[j + 1] = key;
    }

    s_filtered_cm = valid[n / 2];
    return s_filtered_cm;
}
