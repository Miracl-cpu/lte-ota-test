/**
 * @file wifi_manager.h
 * @brief WiFi Manager for CT+HMI System - Station Mode with Auto-Reconnect
 *
 * Provides WiFi station mode connectivity with:
 * - Automatic reconnection on disconnect
 * - RSSI monitoring
 * - Connection status tracking
 * - Callback support for connection events
 */

#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief WiFi connection status
 */
typedef struct {
    bool connected;
    int8_t rssi;          // Signal strength in dBm
    char ip_addr[16];     // IP address string
} wifi_status_t;

/**
 * @brief WiFi configuration
 */
typedef struct {
    char ssid[32];
    char password[64];
    uint32_t retry_interval_ms;  // Reconnect interval (default 5000ms)
} wifi_manager_config_t;

/**
 * @brief Callback type for WiFi connected event
 */
typedef void (*wifi_connected_cb_t)(void);

/**
 * @brief Initialize WiFi in station mode
 * @param config WiFi configuration
 * @return ESP_OK on success
 */
esp_err_t wifi_manager_init(const wifi_manager_config_t *config);

/**
 * @brief Set callback for WiFi connected event
 * @param cb Callback function (called when WiFi connects)
 */
void wifi_manager_set_connected_callback(wifi_connected_cb_t cb);

/**
 * @brief Start WiFi connection (non-blocking)
 * @return ESP_OK on success
 */
esp_err_t wifi_manager_start(void);

/**
 * @brief Stop WiFi connection and disable auto-reconnect.
 * Safe to call multiple times.
 */
void wifi_manager_stop(void);

/**
 * @brief Get current WiFi status
 * @param status Output status structure
 */
void wifi_manager_get_status(wifi_status_t *status);

/**
 * @brief Check if WiFi is connected
 * @return true if connected
 */
bool wifi_manager_is_connected(void);

/**
 * @brief Get current RSSI
 * @return RSSI in dBm, or -100 if not connected
 */
int8_t wifi_manager_get_rssi(void);

#ifdef __cplusplus
}
#endif

#endif // WIFI_MANAGER_H
