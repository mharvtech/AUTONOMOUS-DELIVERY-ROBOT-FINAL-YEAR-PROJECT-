#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// Brings up networking and starts the HTTP server that serves the control
// panel at "/" and the REST API under "/api/...".
//
// The robot always raises its own access point (ROBOT_AP_SSID in web_server.c)
// so it is controllable on a bench with no infrastructure, and additionally
// joins the network configured in `idf.py menuconfig` -> Delivery Robot WiFi
// Configuration if one is set.
//
// Returns ESP_FAIL only if the HTTP server itself could not start. Failing to
// join the configured network is not fatal - the access point is still there.
//
// settings_init() must have run first: it owns nvs_flash_init(), which the
// WiFi driver depends on.
esp_err_t web_server_start(void);

#ifdef __cplusplus
}
#endif
