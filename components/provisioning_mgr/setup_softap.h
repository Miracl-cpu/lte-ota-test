/**
 * @file setup_softap.h
 * @brief SoftAP management for setup mode
 *
 * Creates an open WiFi access point for device configuration.
 * Includes DNS server for captive portal redirection.
 */

#ifndef SETUP_SOFTAP_H
#define SETUP_SOFTAP_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// SoftAP configuration
#define SOFTAP_IP           "192.168.4.1"
#define SOFTAP_NETMASK      "255.255.255.0"
#define SOFTAP_GW           "192.168.4.1"
#define SOFTAP_DHCP_START   "192.168.4.2"
#define SOFTAP_DHCP_END     "192.168.4.10"

/**
 * @brief Start SoftAP for setup mode
 * SSID will be "CT-HMI-XXXX" where XXXX is last 4 of MAC
 * @param ssid_out Buffer to receive generated SSID (min 20 chars)
 * @return ESP_OK on success
 */
esp_err_t setup_softap_start(char *ssid_out);

/**
 * @brief Stop SoftAP
 * @return ESP_OK on success
 */
esp_err_t setup_softap_stop(void);

/**
 * @brief Check if SoftAP is running
 * @return true if active
 */
bool setup_softap_is_active(void);

/**
 * @brief Get number of connected stations
 * @return Number of connected clients
 */
uint8_t setup_softap_get_client_count(void);

/**
 * @brief Get SoftAP IP address
 * @return IP address string "192.168.4.1"
 */
const char* setup_softap_get_ip(void);

/**
 * @brief Start DNS server for captive portal
 * Redirects all DNS queries to the SoftAP IP
 * @return ESP_OK on success
 */
esp_err_t setup_softap_start_dns(void);

/**
 * @brief Stop DNS server
 * @return ESP_OK on success
 */
esp_err_t setup_softap_stop_dns(void);

#ifdef __cplusplus
}
#endif

#endif // SETUP_SOFTAP_H
