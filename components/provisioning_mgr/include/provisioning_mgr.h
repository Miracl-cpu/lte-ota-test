/**
 * @file provisioning_mgr.h
 * @brief Provisioning Manager - Secure setup mode controller
 *
 * Manages the complete provisioning flow:
 * 1. Setup trigger detection (# hold for 5s)
 * 2. Password entry and validation
 * 3. Setup mode activation (SoftAP + Web UI)
 * 4. Configuration save and exit
 */

#ifndef PROVISIONING_MGR_H
#define PROVISIONING_MGR_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// Security configuration
#define SETUP_PASSWORD          "7552"          // 4-digit setup password
#define SETUP_PASSWORD_LEN      4
#define HASH_HOLD_TIME_MS       5000            // 5 seconds to trigger
#define PASSWORD_TIMEOUT_MS     30000           // 30 seconds to enter password
#define MESSAGE_DISPLAY_MS      2000            // 2 seconds for error messages
#define SETUP_INACTIVITY_TIMEOUT_MS 300000      // 5 minutes inactivity auto-exit

// SoftAP configuration
#define SOFTAP_SSID_PREFIX      "CT-HMI-"       // + last 4 of MAC
#define SOFTAP_CHANNEL          6
#define SOFTAP_MAX_CONN         4

/**
 * @brief Provisioning manager states
 */
typedef enum {
    PROV_STATE_NORMAL,              // Normal operation, monitoring for trigger
    PROV_STATE_HASH_HOLDING,        // '#' being held, counting to 5s
    PROV_STATE_PASSWORD_ENTRY,      // Waiting for password input
    PROV_STATE_ACCESS_DENIED,       // Wrong password, showing error
    PROV_STATE_TIMEOUT,             // Password timeout, showing message
    PROV_STATE_ENTERING_SETUP,      // Transitioning to setup mode
    PROV_STATE_SETUP_ACTIVE,        // Setup mode running (SoftAP + Web)
    PROV_STATE_EXITING_SETUP,       // Transitioning back to normal
    PROV_STATE_REBOOTING            // Reboot pending
} prov_state_t;

/**
 * @brief Provisioning manager status (for LCD display)
 */
typedef struct {
    prov_state_t state;
    uint32_t hold_progress_ms;      // How long '#' has been held
    uint32_t timeout_remaining_ms;  // Password entry time remaining
    char password_mask[8];          // "****", "**__", etc.
    char softap_ssid[20];           // "CT-HMI-A3F4"
    char softap_ip[16];             // "192.168.4.1"
    uint8_t connected_clients;      // Number of connected stations
    bool config_saved;              // True if config was just saved
} prov_status_t;

/**
 * @brief LCD update callback type
 * Called when provisioning state changes and LCD needs update
 */
typedef void (*prov_lcd_update_cb_t)(const prov_status_t *status);

/**
 * @brief Configuration for provisioning manager
 */
typedef struct {
    prov_lcd_update_cb_t lcd_callback;  // LCD update callback (required)
} prov_config_t;

/**
 * @brief Initialize provisioning manager
 * @param config Configuration structure
 * @return ESP_OK on success
 */
esp_err_t provisioning_mgr_init(const prov_config_t *config);

/**
 * @brief Process key input for provisioning trigger
 * Call this from HMI task for every key event
 *
 * @param key The key character ('0'-'9', 'A'-'D', '*', '#')
 * @param pressed true if key pressed, false if released
 * @return true if key was consumed by provisioning (don't process normally)
 */
bool provisioning_mgr_process_key(char key, bool pressed);

/**
 * @brief Periodic update function
 * Call this from HMI task every loop iteration (~25-100ms)
 * Handles timeouts, progress updates, etc.
 *
 * @param now_ms Current timestamp in milliseconds
 */
void provisioning_mgr_update(uint32_t now_ms);

/**
 * @brief Get current provisioning state
 * @return Current state enum
 */
prov_state_t provisioning_mgr_get_state(void);

/**
 * @brief Get current provisioning status
 * @param status Pointer to status structure to fill
 */
void provisioning_mgr_get_status(prov_status_t *status);

/**
 * @brief Check if in setup mode (SoftAP active)
 * @return true if setup mode is active
 */
bool provisioning_mgr_is_setup_mode(void);

/**
 * @brief Check if normal operations should run
 * DAQ should always run. WiFi/MQTT should pause during setup.
 * @return true if WiFi/MQTT should be active
 */
bool provisioning_mgr_is_normal_mode(void);

/**
 * @brief Enter setup mode immediately
 *
 * Starts the SoftAP + web configuration portal without requiring the
 * keypad hold/password trigger. Used by the local LCD menu.
 *
 * @return ESP_OK when setup mode starts, otherwise an ESP-IDF error
 */
esp_err_t provisioning_mgr_enter_setup(void);

/**
 * @brief Force exit from setup mode
 * Used when config is saved via web UI
 */
void provisioning_mgr_exit_setup(void);

/**
 * @brief Trigger reboot
 * Schedules a reboot after brief delay
 */
void provisioning_mgr_request_reboot(void);

#ifdef __cplusplus
}
#endif

#endif // PROVISIONING_MGR_H
