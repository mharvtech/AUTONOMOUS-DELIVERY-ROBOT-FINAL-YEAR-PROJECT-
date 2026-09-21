#include "color_sensor.h"
#include <stdlib.h>
#include <string.h>
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_rom_sys.h"

static const char *TAG = "color_sensor";

#define I2C_FREQ_HZ     100000
#define I2C_TIMEOUT_MS  100

// ---------------- TCS34725 registers ----------------
#define TCS34725_ADDRESS      0x29   // fixed - no address-select pin
#define TCS34725_COMMAND_BIT  0x80
#define TCS34725_ENABLE       0x00
#define TCS34725_ENABLE_PON   0x01
#define TCS34725_ENABLE_AEN   0x02
#define TCS34725_ATIME        0x01
#define TCS34725_CONTROL      0x0F
#define TCS34725_ID           0x12
#define TCS34725_CDATAL       0x14
#define TCS34725_RDATAL       0x16
#define TCS34725_GDATAL       0x18
#define TCS34725_BDATAL       0x1A

// After white balancing, a neutral surface sits near 0.333 on all three
// channels. A marker only counts as coloured if its strongest channel beats the
// runner-up by at least this. Raise it if plain wall is reported as a colour;
// lower it if pale markers are missed.
#define COLOR_DOMINANCE_MARGIN 0.06f

struct color_sensor_s {
    i2c_master_bus_handle_t bus;
    i2c_master_dev_handle_t dev;
    int      port;
    char     name[16];
    uint16_t clear_on_line;      // at or below this the surface is the line
    uint16_t clear_on_floor;     // at or above this it is bare floor
    uint16_t wb_r, wb_g, wb_b;   // white reference
};

static esp_err_t tcs_write8(color_sensor_handle_t h, uint8_t reg, uint8_t value)
{
    uint8_t buf[2] = { (uint8_t)(TCS34725_COMMAND_BIT | reg), value };
    return i2c_master_transmit(h->dev, buf, sizeof(buf), pdMS_TO_TICKS(I2C_TIMEOUT_MS));
}

static esp_err_t tcs_read8(color_sensor_handle_t h, uint8_t reg, uint8_t *value)
{
    uint8_t reg_addr = (uint8_t)(TCS34725_COMMAND_BIT | reg);
    return i2c_master_transmit_receive(h->dev, &reg_addr, 1, value, 1,
                                       pdMS_TO_TICKS(I2C_TIMEOUT_MS));
}

static esp_err_t tcs_read16(color_sensor_handle_t h, uint8_t reg, uint16_t *value)
{
    uint8_t reg_addr = (uint8_t)(TCS34725_COMMAND_BIT | reg);
    uint8_t data[2];
    esp_err_t err = i2c_master_transmit_receive(h->dev, &reg_addr, 1, data, 2,
                                                pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    if (err == ESP_OK) {
        *value = (uint16_t)data[0] | ((uint16_t)data[1] << 8);
    }
    return err;
}

esp_err_t color_sensor_init(const color_sensor_config_t *cfg,
                            color_sensor_handle_t *out_handle)
{
    if (!cfg || !out_handle) return ESP_ERR_INVALID_ARG;
    *out_handle = NULL;

    color_sensor_handle_t h = calloc(1, sizeof(struct color_sensor_s));
    if (!h) return ESP_ERR_NO_MEM;

    h->port = cfg->i2c_port;
    strncpy(h->name, cfg->name ? cfg->name : "color", sizeof(h->name) - 1);
    h->clear_on_line  = 300;
    h->clear_on_floor = 1500;
    h->wb_r = h->wb_g = h->wb_b = 1000;   // equal = raw ratios until calibrated

    i2c_master_bus_config_t bus_config = {
        .i2c_port = (i2c_port_num_t)cfg->i2c_port,
        .sda_io_num = cfg->sda_gpio,
        .scl_io_num = cfg->scl_gpio,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    esp_err_t err = i2c_new_master_bus(&bus_config, &h->bus);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "%s: I2C%d bus create failed: %s",
                 h->name, cfg->i2c_port, esp_err_to_name(err));
        free(h);
        return err;
    }

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = TCS34725_ADDRESS,
        .scl_speed_hz = I2C_FREQ_HZ,
    };
    err = i2c_master_bus_add_device(h->bus, &dev_config, &h->dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "%s: could not add TCS34725: %s", h->name, esp_err_to_name(err));
        i2c_del_master_bus(h->bus);
        free(h);
        return err;
    }

    uint8_t id = 0;
    err = tcs_read8(h, TCS34725_ID, &id);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "%s: no response on I2C%d (SDA%d/SCL%d) - check wiring and 3V3.",
                 h->name, cfg->i2c_port, cfg->sda_gpio, cfg->scl_gpio);
        i2c_master_bus_rm_device(h->dev);
        i2c_del_master_bus(h->bus);
        free(h);
        return err;
    }
    ESP_LOGI(TAG, "%s: TCS34725 on I2C%d (SDA%d/SCL%d), ID 0x%02X (expect 0x44 or 0x4D)",
             h->name, cfg->i2c_port, cfg->sda_gpio, cfg->scl_gpio, id);

    tcs_write8(h, TCS34725_ATIME, cfg->atime);
    tcs_write8(h, TCS34725_CONTROL, cfg->gain);
    tcs_write8(h, TCS34725_ENABLE, TCS34725_ENABLE_PON);
    // The datasheet wants 2.4ms between PON and AEN. pdMS_TO_TICKS(3) is zero
    // ticks at a 100Hz tick and would not wait at all, so busy-wait the exact
    // time instead - this runs once, at boot.
    esp_rom_delay_us(3000);
    tcs_write8(h, TCS34725_ENABLE, TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN);
    vTaskDelay(pdMS_TO_TICKS(200));   // let the first integration cycle complete

    *out_handle = h;
    return ESP_OK;
}

static esp_err_t read_raw(color_sensor_handle_t h,
                          uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *c)
{
    if (!h) return ESP_ERR_INVALID_STATE;
    esp_err_t err;
    err = tcs_read16(h, TCS34725_CDATAL, c); if (err != ESP_OK) return err;
    err = tcs_read16(h, TCS34725_RDATAL, r); if (err != ESP_OK) return err;
    err = tcs_read16(h, TCS34725_GDATAL, g); if (err != ESP_OK) return err;
    err = tcs_read16(h, TCS34725_BDATAL, b); if (err != ESP_OK) return err;
    return ESP_OK;
}

// Ratio based, not absolute: a red marker in bright light and the same marker
// in shadow give very different raw counts but almost identical channel
// proportions. White balancing first cancels the sensor's own channel
// imbalance; then the strongest channel wins only if it beats the runner-up by
// COLOR_DOMINANCE_MARGIN, otherwise the surface is neutral and the answer is
// NONE.
static color_id_t classify(color_sensor_handle_t h,
                           uint16_t r, uint16_t g, uint16_t b, uint16_t c)
{
    if (c <= h->clear_on_line) {
        // Too little light coming back to call a hue. Under the centre sensor
        // that is the guide line; under the front sensor it is an unlit wall,
        // and main.c never treats black as a destination.
        return COLOR_BLACK;
    }

    float wr = h->wb_r ? h->wb_r : 1;
    float wg = h->wb_g ? h->wb_g : 1;
    float wb = h->wb_b ? h->wb_b : 1;

    float rn = (float)r / wr;
    float gn = (float)g / wg;
    float bn = (float)b / wb;

    float total = rn + gn + bn;
    if (total < 0.0001f) return COLOR_NONE;

    float rp = rn / total, gp = gn / total, bp = bn / total;

    float top;
    color_id_t winner;
    if (rp >= gp && rp >= bp)      { top = rp; winner = COLOR_RED; }
    else if (gp >= rp && gp >= bp) { top = gp; winner = COLOR_GREEN; }
    else                           { top = bp; winner = COLOR_BLUE; }

    float second = -1.0f;
    if (winner != COLOR_RED   && rp > second) second = rp;
    if (winner != COLOR_GREEN && gp > second) second = gp;
    if (winner != COLOR_BLUE  && bp > second) second = bp;

    if (top - second < COLOR_DOMINANCE_MARGIN) {
        return COLOR_NONE;   // neutral surface
    }
    return winner;
}

color_reading_t color_sensor_read(color_sensor_handle_t h)
{
    color_reading_t out = { .valid = false, .r = 0, .g = 0, .b = 0, .c = 0,
                            .color = COLOR_NONE, .on_line = false, .darkness = 0 };
    if (!h) return out;

    uint16_t r, g, b, c;
    if (read_raw(h, &r, &g, &b, &c) != ESP_OK) {
        return out;
    }

    out.valid = true;
    out.r = r; out.g = g; out.b = b; out.c = c;
    out.color = classify(h, r, g, b, c);

    if (c <= h->clear_on_line) {
        out.on_line  = true;
        out.darkness = 100;
    } else if (c >= h->clear_on_floor) {
        out.darkness = 0;
    } else {
        // Linear ramp between the two calibration points. Only the diagnostics
        // panel uses this; the steering itself only needs on_line.
        out.darkness = (int)(100.0f * (float)(h->clear_on_floor - c) /
                                      (float)(h->clear_on_floor - h->clear_on_line));
    }
    return out;
}

esp_err_t color_sensor_calibrate_white(color_sensor_handle_t h)
{
    if (!h) return ESP_ERR_INVALID_STATE;
    uint16_t r, g, b, c;
    if (read_raw(h, &r, &g, &b, &c) != ESP_OK) return ESP_FAIL;

    if (r == 0 || g == 0 || b == 0 || c <= h->clear_on_line) {
        ESP_LOGW(TAG, "%s: white sample too dark (R%u G%u B%u C%u). Point it at "
                      "white paper in normal room light.", h->name, r, g, b, c);
        return ESP_FAIL;
    }
    h->wb_r = r; h->wb_g = g; h->wb_b = b;
    ESP_LOGI(TAG, "%s: white balance set to R%u G%u B%u (clear %u)",
             h->name, r, g, b, c);
    return ESP_OK;
}

void color_sensor_set_white_balance(color_sensor_handle_t h,
                                    uint16_t r, uint16_t g, uint16_t b)
{
    if (!h || r == 0 || g == 0 || b == 0) return;
    h->wb_r = r; h->wb_g = g; h->wb_b = b;
}

void color_sensor_get_white_balance(color_sensor_handle_t h,
                                    uint16_t *r, uint16_t *g, uint16_t *b)
{
    if (!h) return;
    if (r) *r = h->wb_r;
    if (g) *g = h->wb_g;
    if (b) *b = h->wb_b;
}

void color_sensor_set_line_window(color_sensor_handle_t h,
                                  uint16_t clear_on_line, uint16_t clear_on_floor)
{
    if (!h) return;
    if (clear_on_floor <= clear_on_line) {
        ESP_LOGW(TAG, "%s: ignoring line window - floor (%u) must exceed line (%u).",
                 h->name, clear_on_floor, clear_on_line);
        return;
    }
    h->clear_on_line  = clear_on_line;
    h->clear_on_floor = clear_on_floor;
    ESP_LOGI(TAG, "%s: line window set - line<=%u, floor>=%u",
             h->name, clear_on_line, clear_on_floor);
}

void color_sensor_get_line_window(color_sensor_handle_t h,
                                  uint16_t *clear_on_line, uint16_t *clear_on_floor)
{
    if (!h) return;
    if (clear_on_line)  *clear_on_line  = h->clear_on_line;
    if (clear_on_floor) *clear_on_floor = h->clear_on_floor;
}

esp_err_t color_sensor_calibrate_line(color_sensor_handle_t h)
{
    if (!h) return ESP_ERR_INVALID_STATE;
    uint16_t r, g, b, c;
    if (read_raw(h, &r, &g, &b, &c) != ESP_OK) return ESP_FAIL;

    // Sit the threshold a little above the measured value so ordinary sensor
    // noise cannot drop the reading out of "on line" mid-run.
    uint16_t line = (uint16_t)(c + c / 10 + 5);
    if (line >= h->clear_on_floor) {
        ESP_LOGW(TAG, "%s: line sample (%u) is not darker than the stored floor (%u).",
                 h->name, c, h->clear_on_floor);
        return ESP_FAIL;
    }
    h->clear_on_line = line;
    ESP_LOGI(TAG, "%s: line calibrated at clear=%u -> threshold %u", h->name, c, line);
    return ESP_OK;
}

esp_err_t color_sensor_calibrate_floor(color_sensor_handle_t h)
{
    if (!h) return ESP_ERR_INVALID_STATE;
    uint16_t r, g, b, c;
    if (read_raw(h, &r, &g, &b, &c) != ESP_OK) return ESP_FAIL;

    uint16_t floor_val = (c > 10) ? (uint16_t)(c - c / 10) : c;
    if (floor_val <= h->clear_on_line) {
        ESP_LOGW(TAG, "%s: floor sample (%u) is not brighter than the stored line (%u).",
                 h->name, c, h->clear_on_line);
        return ESP_FAIL;
    }
    h->clear_on_floor = floor_val;
    ESP_LOGI(TAG, "%s: floor calibrated at clear=%u -> threshold %u",
             h->name, c, floor_val);
    return ESP_OK;
}

const char *color_name(color_id_t color)
{
    switch (color) {
        case COLOR_RED:   return "RED";
        case COLOR_GREEN: return "GREEN";
        case COLOR_BLUE:  return "BLUE";
        case COLOR_BLACK: return "BLACK";
        default:          return "NONE";
    }
}
