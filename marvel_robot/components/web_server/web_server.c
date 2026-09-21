#include "web_server.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_log.h"
#include "esp_http_server.h"
#include "mdns.h"
#include "cJSON.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

#include "robot_state.h"
#include "motor.h"

static const char *TAG = "web_server";

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1
#define WIFI_MAX_RETRY     8

// The robot answers to http://marvel.local/ on the LAN, so the IP address
// never has to be typed in.
#define ROBOT_HOSTNAME      "marvel"
#define ROBOT_INSTANCE_NAME "Marvel Delivery Robot"

// The robot's own access point, always on, so the control panel is reachable
// on a bench or at a demo where there is no WiFi to join: connect a phone to
// it and open http://192.168.4.1/. A WPA2 password must be at least 8
// characters; an empty one gives an open network, which lets anyone in range
// drive the robot.
#define ROBOT_AP_SSID     "marvel-robot"
#define ROBOT_AP_PASSWORD "marvel1234"
#define ROBOT_AP_CHANNEL  1
#define ROBOT_AP_MAX_CONN 4

static EventGroupHandle_t s_wifi_event_group;
static int  s_retry_num = 0;
static char s_ip_str[16] = "0.0.0.0";
static bool s_sta_connected = false;

// The control panel HTML, embedded into the firmware binary from
// components/web_server/www/index.html (see CMakeLists.txt).
extern const uint8_t index_html_start[] asm("_binary_index_html_start");
extern const uint8_t index_html_end[]   asm("_binary_index_html_end");

// =========================================================
//                          WIFI
// =========================================================
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        s_sta_connected = false;
        if (s_retry_num < WIFI_MAX_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGW(TAG, "Retrying WiFi connection (%d/%d)...", s_retry_num, WIFI_MAX_RETRY);
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *) event_data;
        snprintf(s_ip_str, sizeof(s_ip_str), IPSTR, IP2STR(&event->ip_info.ip));
        ESP_LOGI(TAG, "Joined network. Robot IP address: %s", s_ip_str);
        s_retry_num = 0;
        s_sta_connected = true;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STACONNECTED) {
        ESP_LOGI(TAG, "A device joined the robot's own access point.");
    }
}

// AP and STA together rather than STA with a fallback, so the access point is
// never lost: if the configured network is out of range, wrong, or simply down
// at a demo, the robot is still drivable instead of unreachable until it is
// reflashed.
static void wifi_init_apsta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler, NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler, NULL,
                                                        &instance_got_ip));

    wifi_config_t ap_config = {
        .ap = {
            .channel = ROBOT_AP_CHANNEL,
            .max_connection = ROBOT_AP_MAX_CONN,
            .authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    strncpy((char *)ap_config.ap.ssid, ROBOT_AP_SSID, sizeof(ap_config.ap.ssid) - 1);
    ap_config.ap.ssid_len = strlen(ROBOT_AP_SSID);
    strncpy((char *)ap_config.ap.password, ROBOT_AP_PASSWORD,
            sizeof(ap_config.ap.password) - 1);
    if (strlen(ROBOT_AP_PASSWORD) == 0) {
        ap_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    wifi_config_t sta_config = {
        .sta = { .threshold.authmode = WIFI_AUTH_WPA2_PSK },
    };
    strncpy((char *)sta_config.sta.ssid, CONFIG_WIFI_SSID,
            sizeof(sta_config.sta.ssid) - 1);
    strncpy((char *)sta_config.sta.password, CONFIG_WIFI_PASSWORD,
            sizeof(sta_config.sta.password) - 1);

    bool have_sta = strlen(CONFIG_WIFI_SSID) > 0;

    ESP_ERROR_CHECK(esp_wifi_set_mode(have_sta ? WIFI_MODE_APSTA : WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_config));
    if (have_sta) {
        ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta_config));
    }
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Access point up: SSID \"%s\" -> http://192.168.4.1/", ROBOT_AP_SSID);

    if (!have_sta) {
        ESP_LOGW(TAG, "No station SSID configured - access point only.");
        return;
    }

    ESP_LOGI(TAG, "Joining WiFi SSID: %s", CONFIG_WIFI_SSID);
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE, pdFALSE, pdMS_TO_TICKS(20000));
    if (bits & WIFI_CONNECTED_BIT) return;

    ESP_LOGW(TAG, "Could not join \"%s\" - use the \"%s\" access point instead "
                  "(http://192.168.4.1/).", CONFIG_WIFI_SSID, ROBOT_AP_SSID);
}

static esp_err_t mdns_start(void)
{
    esp_err_t err = mdns_init();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "mDNS init failed (%s) - use the IP address instead.",
                 esp_err_to_name(err));
        return err;
    }
    mdns_hostname_set(ROBOT_HOSTNAME);
    mdns_instance_name_set(ROBOT_INSTANCE_NAME);
    mdns_service_add(NULL, "_http", "_tcp", 80, NULL, 0);

    ESP_LOGI(TAG, "Control panel: http://%s.local/  (or http://%s/, or "
                  "http://192.168.4.1/ on the robot's own AP)", ROBOT_HOSTNAME, s_ip_str);
    return ESP_OK;
}

// =========================================================
//                      HTTP helpers
// =========================================================
static void set_cors(httpd_req_t *req)
{
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "GET, POST, OPTIONS");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "Content-Type");
}

static cJSON *parse_body(httpd_req_t *req)
{
    if (req->content_len <= 0 || req->content_len > 511) return NULL;
    char buf[512];
    int received = httpd_req_recv(req, buf, req->content_len);
    if (received <= 0) return NULL;
    buf[received] = '\0';
    return cJSON_Parse(buf);
}

static esp_err_t send_json_ok(httpd_req_t *req)
{
    httpd_resp_set_type(req, "application/json");
    set_cors(req);
    httpd_resp_send(req, "{\"ok\":true}", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static esp_err_t send_json_error(httpd_req_t *req, const char *msg)
{
    httpd_resp_set_status(req, "400 Bad Request");
    httpd_resp_set_type(req, "application/json");
    set_cors(req);
    char buf[192];
    snprintf(buf, sizeof(buf), "{\"ok\":false,\"error\":\"%s\"}", msg);
    httpd_resp_send(req, buf, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static esp_err_t send_cjson(httpd_req_t *req, cJSON *root)
{
    char *json_str = cJSON_PrintUnformatted(root);
    httpd_resp_set_type(req, "application/json");
    set_cors(req);
    if (json_str) {
        httpd_resp_send(req, json_str, HTTPD_RESP_USE_STRLEN);
        free(json_str);
    } else {
        httpd_resp_send(req, "{}", HTTPD_RESP_USE_STRLEN);
    }
    cJSON_Delete(root);
    return ESP_OK;
}

static cJSON *color_reading_json(const color_reading_t *reading)
{
    cJSON *obj = cJSON_CreateObject();
    cJSON_AddBoolToObject(obj, "valid", reading->valid);
    cJSON_AddStringToObject(obj, "color", color_to_lower_string(reading->color));
    cJSON_AddNumberToObject(obj, "r", reading->r);
    cJSON_AddNumberToObject(obj, "g", reading->g);
    cJSON_AddNumberToObject(obj, "b", reading->b);
    cJSON_AddNumberToObject(obj, "c", reading->c);
    cJSON_AddBoolToObject(obj, "on_line", reading->on_line);
    cJSON_AddNumberToObject(obj, "darkness", reading->darkness);
    return obj;
}

static cJSON *ir_json(const robot_telemetry_t *t)
{
    cJSON *ir = cJSON_CreateObject();
    cJSON_AddBoolToObject(ir, "left", t->ir_left);
    cJSON_AddBoolToObject(ir, "right", t->ir_right);
    cJSON_AddNumberToObject(ir, "left_level", t->ir_left_level);
    cJSON_AddNumberToObject(ir, "right_level", t->ir_right_level);
    cJSON_AddNumberToObject(ir, "line_level", t->ir_line_level);
    return ir;
}

// =========================================================
//                     Route handlers
// =========================================================
static esp_err_t root_get_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, (const char *)index_html_start,
                    index_html_end - index_html_start);
    return ESP_OK;
}

// Browsers send a preflight OPTIONS before any cross-origin POST carrying a
// JSON content type. Without this the panel works when served by the robot
// (same origin, no preflight) but every button fails silently when the same
// file is opened from disk.
static esp_err_t options_handler(httpd_req_t *req)
{
    set_cors(req);
    httpd_resp_set_hdr(req, "Access-Control-Max-Age", "600");
    httpd_resp_set_status(req, "204 No Content");
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
}

static esp_err_t info_get_handler(httpd_req_t *req)
{
    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "hostname", ROBOT_HOSTNAME ".local");
    cJSON_AddStringToObject(root, "ip", s_ip_str);
    cJSON_AddBoolToObject(root, "station_connected", s_sta_connected);
    cJSON_AddStringToObject(root, "ap_ssid", ROBOT_AP_SSID);
    return send_cjson(req, root);
}

static esp_err_t status_get_handler(httpd_req_t *req)
{
    robot_telemetry_t t;
    robot_state_get_telemetry(&t);

    bool cal_ok = false;
    char cal_msg[CAL_MESSAGE_LEN];
    robot_state_get_calibration_result(&cal_ok, cal_msg, sizeof(cal_msg));

    cJSON *root = cJSON_CreateObject();
    cJSON_AddBoolToObject(root, "connected", true);
    cJSON_AddStringToObject(root, "mode",
        robot_state_get_mode() == ROBOT_MODE_AUTONOMOUS ? "autonomous" : "manual");
    cJSON_AddStringToObject(root, "state", t.state);
    cJSON_AddNumberToObject(root, "distance_cm", t.distance_cm);
    cJSON_AddStringToObject(root, "target", color_to_lower_string(t.target));
    cJSON_AddBoolToObject(root, "mission_running", t.mission_running);
    cJSON_AddBoolToObject(root, "arrived", t.arrived);
    cJSON_AddStringToObject(root, "arrived_via", t.arrived_via);

    cJSON_AddItemToObject(root, "front", color_reading_json(&t.front));
    cJSON_AddItemToObject(root, "ir", ir_json(&t));

    cJSON *line = cJSON_CreateObject();
    cJSON_AddNumberToObject(line, "turn_dir", t.turn_dir);
    cJSON_AddNumberToObject(line, "black_ms", t.black_ms);
    cJSON_AddItemToObject(root, "line", line);

    cJSON *speed = cJSON_CreateObject();
    cJSON_AddNumberToObject(speed, "left", t.left_speed);
    cJSON_AddNumberToObject(speed, "right", t.right_speed);
    cJSON_AddItemToObject(root, "speed", speed);

    cJSON *cal = cJSON_CreateObject();
    cJSON_AddBoolToObject(cal, "ok", cal_ok);
    cJSON_AddStringToObject(cal, "message", cal_msg);
    cJSON_AddNumberToObject(cal, "wb_r", t.wb_r);
    cJSON_AddNumberToObject(cal, "wb_g", t.wb_g);
    cJSON_AddNumberToObject(cal, "wb_b", t.wb_b);
    cJSON_AddItemToObject(root, "calibration", cal);

    return send_cjson(req, root);
}

// Raw sensor view, so wiring and calibration can be checked from a phone
// instead of over `idf.py monitor` with the robot tethered to a laptop.
static esp_err_t sensors_get_handler(httpd_req_t *req)
{
    robot_telemetry_t t;
    robot_state_get_telemetry(&t);

    cJSON *root = cJSON_CreateObject();
    cJSON_AddItemToObject(root, "front", color_reading_json(&t.front));
    cJSON_AddItemToObject(root, "ir", ir_json(&t));
    cJSON_AddNumberToObject(root, "distance_cm", t.distance_cm);

    cJSON *wb = cJSON_CreateObject();
    cJSON_AddNumberToObject(wb, "r", t.wb_r);
    cJSON_AddNumberToObject(wb, "g", t.wb_g);
    cJSON_AddNumberToObject(wb, "b", t.wb_b);
    cJSON_AddItemToObject(root, "white_balance", wb);

    return send_cjson(req, root);
}

static esp_err_t mode_post_handler(httpd_req_t *req)
{
    cJSON *body = parse_body(req);
    if (!body) return send_json_error(req, "invalid_body");

    cJSON *mode_item = cJSON_GetObjectItem(body, "mode");
    if (!cJSON_IsString(mode_item)) {
        cJSON_Delete(body);
        return send_json_error(req, "missing_mode");
    }

    bool to_manual = strcmp(mode_item->valuestring, "manual") == 0;
    robot_state_set_mode(to_manual ? ROBOT_MODE_MANUAL : ROBOT_MODE_AUTONOMOUS);

    // Changing mode must never leave a latched command behind.
    robot_state_set_manual_command(MANUAL_STOP, 0);
    motor_stop();

    cJSON_Delete(body);
    return send_json_ok(req);
}

static esp_err_t autonomous_start_post_handler(httpd_req_t *req)
{
    cJSON *body = parse_body(req);
    if (!body) return send_json_error(req, "invalid_body");

    cJSON *color_item = cJSON_GetObjectItem(body, "target_color");
    if (!cJSON_IsString(color_item)) {
        cJSON_Delete(body);
        return send_json_error(req, "missing_target_color");
    }

    color_id_t target = color_from_string(color_item->valuestring);
    cJSON_Delete(body);
    if (target == COLOR_NONE) return send_json_error(req, "invalid_color");

    robot_state_start_mission(target);
    ESP_LOGI(TAG, "Mission started via API - target colour: %s",
             color_to_lower_string(target));
    return send_json_ok(req);
}

static esp_err_t autonomous_stop_post_handler(httpd_req_t *req)
{
    robot_state_stop_mission();
    motor_stop();
    ESP_LOGI(TAG, "Mission stopped via API.");
    return send_json_ok(req);
}

static esp_err_t manual_move_post_handler(httpd_req_t *req)
{
    cJSON *body = parse_body(req);
    if (!body) return send_json_error(req, "invalid_body");

    cJSON *dir_item = cJSON_GetObjectItem(body, "direction");
    cJSON *pwm_item = cJSON_GetObjectItem(body, "pwm");
    if (!cJSON_IsString(dir_item)) {
        cJSON_Delete(body);
        return send_json_error(req, "missing_direction");
    }

    int pwm = cJSON_IsNumber(pwm_item) ? pwm_item->valueint : 0;
    if (pwm < 0) pwm = 0;
    if (pwm > 255) pwm = 255;

    manual_direction_t dir = manual_direction_from_string(dir_item->valuestring);
    cJSON_Delete(body);

    if (robot_state_get_mode() != ROBOT_MODE_MANUAL) {
        return send_json_error(req, "not_in_manual_mode");
    }

    // The control loop latches this until the next command arrives, so this
    // same call doubles as the browser's keep-alive heartbeat.
    robot_state_set_manual_command(dir, pwm);
    return send_json_ok(req);
}

// Calibration is only requested here; the control loop performs it, because
// only that task may touch the I2C bus and the motors.
static esp_err_t calibrate_post_handler(httpd_req_t *req)
{
    cJSON *body = parse_body(req);
    if (!body) return send_json_error(req, "invalid_body");

    cJSON *target_item = cJSON_GetObjectItem(body, "target");
    if (!cJSON_IsString(target_item)) {
        cJSON_Delete(body);
        return send_json_error(req, "missing_target");
    }

    calibration_request_t request = CAL_NONE;
    const char *target = target_item->valuestring;
    if      (strcmp(target, "ir_floor") == 0) request = CAL_IR_FLOOR;
    else if (strcmp(target, "white") == 0)    request = CAL_WHITE;
    else if (strcmp(target, "forget") == 0)   request = CAL_FORGET;
    cJSON_Delete(body);

    if (request == CAL_NONE) return send_json_error(req, "invalid_target");

    robot_state_request_calibration(request);
    return send_json_ok(req);
}

static esp_err_t estop_post_handler(httpd_req_t *req)
{
    robot_state_estop();
    motor_brake();
    ESP_LOGW(TAG, "EMERGENCY STOP triggered via API.");
    return send_json_ok(req);
}

// =========================================================
//                     Server startup
// =========================================================
static httpd_handle_t start_httpd(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_uri_handlers = 16;
    config.stack_size = 8192;      // cJSON building the status object needs room
    config.uri_match_fn = httpd_uri_match_wildcard;
    config.lru_purge_enable = true;

    httpd_handle_t server = NULL;
    if (httpd_start(&server, &config) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start HTTP server.");
        return NULL;
    }

    httpd_uri_t routes[] = {
        { .uri = "/",                     .method = HTTP_GET,     .handler = root_get_handler },
        { .uri = "/api/info",             .method = HTTP_GET,     .handler = info_get_handler },
        { .uri = "/api/status",           .method = HTTP_GET,     .handler = status_get_handler },
        { .uri = "/api/sensors",          .method = HTTP_GET,     .handler = sensors_get_handler },
        { .uri = "/api/mode",             .method = HTTP_POST,    .handler = mode_post_handler },
        { .uri = "/api/autonomous/start", .method = HTTP_POST,    .handler = autonomous_start_post_handler },
        { .uri = "/api/autonomous/stop",  .method = HTTP_POST,    .handler = autonomous_stop_post_handler },
        { .uri = "/api/manual/move",      .method = HTTP_POST,    .handler = manual_move_post_handler },
        { .uri = "/api/calibrate",        .method = HTTP_POST,    .handler = calibrate_post_handler },
        { .uri = "/api/estop",            .method = HTTP_POST,    .handler = estop_post_handler },
        { .uri = "/api/*",                .method = HTTP_OPTIONS, .handler = options_handler },
    };

    for (size_t i = 0; i < sizeof(routes) / sizeof(routes[0]); i++) {
        httpd_register_uri_handler(server, &routes[i]);
    }
    return server;
}

esp_err_t web_server_start(void)
{
    // NVS is already up: settings_init() owns nvs_flash_init(), and the WiFi
    // driver needs it. Initialising it twice here would return
    // ESP_ERR_INVALID_STATE on IDF 6.0.
    wifi_init_apsta();

    mdns_start();   // not fatal if it fails - the IP still works

    if (start_httpd() == NULL) return ESP_FAIL;

    ESP_LOGI(TAG, "Web server ready.");
    return ESP_OK;
}
