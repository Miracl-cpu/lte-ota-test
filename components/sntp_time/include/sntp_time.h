/**
 * @file sntp_time.h
 * @brief SNTP Time Synchronization for CT+HMI System
 *
 * Provides NTP time synchronization with:
 * - Configurable timezone
 * - Sync status checking
 * - Blocking wait for sync option
 */

#ifndef SNTP_TIME_H
#define SNTP_TIME_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize and start SNTP time synchronization
 * @param timezone Timezone string, e.g., "IST-5:30" for India
 * @return ESP_OK on success
 */
esp_err_t sntp_time_init(const char *timezone);

/**
 * @brief Set process timezone without starting SNTP
 * @param timezone Timezone string, e.g. "IST-5:30" for India
 * @return ESP_OK on success
 */
esp_err_t sntp_time_set_timezone(const char *timezone);

/**
 * @brief Check if time has been synchronized
 * @return true if system time is valid (SNTP or LTE modem clock)
 */
bool sntp_time_is_synced(void);

/**
 * @brief Wait for time sync with timeout
 * @param timeout_ms Maximum time to wait
 * @return true if synced within timeout
 */
bool sntp_time_wait_for_sync(uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif // SNTP_TIME_H
