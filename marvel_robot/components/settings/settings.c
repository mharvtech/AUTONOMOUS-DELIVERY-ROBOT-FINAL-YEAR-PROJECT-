#include "settings.h"
#include <stdio.h>
#include <string.h>
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"

static const char *TAG = "settings";
static const char *NAMESPACE = "robot";

#define KEY_IR_LEVEL  "ir_level"
#define KEY_LINE_DARK "ln_dark"
#define KEY_LINE_LITE "ln_lite"

static bool s_ready = false;

esp_err_t settings_init(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS partition needs erasing - doing that now.");
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS init failed (%s) - calibration will not persist.",
                 esp_err_to_name(err));
        return err;
    }
    s_ready = true;
    return ESP_OK;
}

static bool load_u16(const char *key, uint16_t *out)
{
    if (!s_ready) return false;
    nvs_handle_t handle;
    if (nvs_open(NAMESPACE, NVS_READONLY, &handle) != ESP_OK) return false;
    uint16_t value = 0;
    esp_err_t err = nvs_get_u16(handle, key, &value);
    nvs_close(handle);
    if (err != ESP_OK) return false;
    *out = value;
    return true;
}

static esp_err_t save_u16(const char *key, uint16_t value)
{
    if (!s_ready) return ESP_FAIL;
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) return err;
    err = nvs_set_u16(handle, key, value);
    if (err == ESP_OK) err = nvs_commit(handle);
    nvs_close(handle);
    return err;
}

// NVS keys are capped at 15 characters, so the per-sensor white balance keys
// are built from a one-letter tag: "wb_f_r", "wb_b_g" and so on.
static void wb_key(char *out, size_t len, const char *which, char channel)
{
    snprintf(out, len, "wb_%s_%c", which, channel);
}

bool settings_load_ir_line_level(int *line_level)
{
    uint16_t value;
    if (!load_u16(KEY_IR_LEVEL, &value)) return false;
    *line_level = value ? 1 : 0;
    ESP_LOGI(TAG, "Restored IR polarity: line reads %s.", *line_level ? "HIGH" : "LOW");
    return true;
}

esp_err_t settings_save_ir_line_level(int line_level)
{
    return save_u16(KEY_IR_LEVEL, line_level ? 1 : 0);
}

bool settings_load_line_window(uint16_t *on_line, uint16_t *on_floor)
{
    uint16_t a, b;
    if (!load_u16(KEY_LINE_DARK, &a)) return false;
    if (!load_u16(KEY_LINE_LITE, &b)) return false;
    if (b <= a) return false;
    *on_line = a;
    *on_floor = b;
    ESP_LOGI(TAG, "Restored line window: line<=%u, floor>=%u", a, b);
    return true;
}

esp_err_t settings_save_line_window(uint16_t on_line, uint16_t on_floor)
{
    esp_err_t err = save_u16(KEY_LINE_DARK, on_line);
    if (err == ESP_OK) err = save_u16(KEY_LINE_LITE, on_floor);
    return err;
}

bool settings_load_white_balance(const char *which, uint16_t *r, uint16_t *g, uint16_t *b)
{
    char key[16];
    uint16_t vr, vg, vb;
    wb_key(key, sizeof(key), which, 'r'); if (!load_u16(key, &vr)) return false;
    wb_key(key, sizeof(key), which, 'g'); if (!load_u16(key, &vg)) return false;
    wb_key(key, sizeof(key), which, 'b'); if (!load_u16(key, &vb)) return false;
    if (vr == 0 || vg == 0 || vb == 0) return false;
    *r = vr; *g = vg; *b = vb;
    ESP_LOGI(TAG, "Restored white balance [%s]: R%u G%u B%u", which, vr, vg, vb);
    return true;
}

esp_err_t settings_save_white_balance(const char *which, uint16_t r, uint16_t g, uint16_t b)
{
    char key[16];
    wb_key(key, sizeof(key), which, 'r');
    esp_err_t err = save_u16(key, r);
    if (err == ESP_OK) { wb_key(key, sizeof(key), which, 'g'); err = save_u16(key, g); }
    if (err == ESP_OK) { wb_key(key, sizeof(key), which, 'b'); err = save_u16(key, b); }
    return err;
}

esp_err_t settings_forget(void)
{
    if (!s_ready) return ESP_FAIL;
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) return err;
    err = nvs_erase_all(handle);
    if (err == ESP_OK) err = nvs_commit(handle);
    nvs_close(handle);
    ESP_LOGW(TAG, "Stored calibration erased.");
    return err;
}
