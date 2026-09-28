/**
 * @file setup_webserver.h
 * @brief HTTP web server for configuration UI
 *
 * Provides REST API endpoints for device configuration and
 * serves the web-based configuration interface.
 */

#ifndef SETUP_WEBSERVER_H
#define SETUP_WEBSERVER_H

#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Callback when configuration is saved via web UI
 */
typedef void (*webserver_config_saved_cb_t)(void);

/**
 * @brief Start HTTP server for configuration
 * @param on_config_saved Callback when config saved
 * @return ESP_OK on success
 */
esp_err_t setup_webserver_start(webserver_config_saved_cb_t on_config_saved);

/**
 * @brief Stop HTTP server
 * @return ESP_OK on success
 */
esp_err_t setup_webserver_stop(void);

/**
 * @brief Check if webserver is running
 * @return true if active
 */
bool setup_webserver_is_active(void);

#ifdef __cplusplus
}
#endif

#endif // SETUP_WEBSERVER_H
