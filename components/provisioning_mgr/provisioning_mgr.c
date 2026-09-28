/**
 * @file provisioning_mgr.c
 * @brief Provisioning Manager implementation
 *
 * Orchestrates all provisioning sub-components:
 * - Setup trigger detection
 * - SoftAP management
 * - Web server
 * - State transitions
 */

#include "provisioning_mgr.h"
#include "setup_trigger.h"
#include "setup_softap.h"
#include "setup_webserver.h"
#include "nvs_config.h"
#include "log_config.h"
#undef LOG_LOCAL_LEVEL
#define LOG_LOCAL_LEVEL LOG_LEVEL_PROV_MGR
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "prov_mgr";

// Internal state
static prov_config_t s_config;
static prov_status_t s_status;
static bool s_initialized = false;
static bool s_reboot_pending = false;
static uint32_t s_reboot_time_ms = 0;

// Reboot delay after config saved
#define REBOOT_DELAY_MS     2000

/**
 * @brief Update status structure and notify LCD callback
 * @param now_ms Current timestamp
 * @param force If true, always call LCD callback (for periodic updates)
 */
static void update_status_and_notify(uint32_t now_ms, bool force)
{
    prov_state_t state = setup_trigger_get_state();

    // Notify if state changed OR force is true (for progress/timeout updates)
    bool should_notify = force || (state != s_status.state);

    s_status.state = state;
    s_status.hold_progress_ms = setup_trigger_get_hold_progress();
    s_status.timeout_remaining_ms = setup_trigger_get_timeout_remaining(now_ms);
    setup_trigger_get_password_mask(s_status.password_mask, sizeof(s_status.password_mask));

    if (setup_softap_is_active()) {
        s_status.connected_clients = setup_softap_get_client_count();
        // Reset inactivity timer while clients are connected
        if (s_status.connected_clients > 0) {
            setup_trigger_reset_activity(now_ms);
        }
    }

    // Notify callback on state change or when forced
    if (should_notify && s_config.lcd_callback) {
        s_config.lcd_callback(&s_status);
    }
}

/**
 * @brief Callback when configuration is saved via web UI
 */
static void on_config_saved(void)
{
    ESP_LOGI(TAG, "Configuration saved via web UI, scheduling reboot");
    s_status.config_saved = true;
    s_reboot_pending = true;
    s_reboot_time_ms = (uint32_t)(esp_timer_get_time() / 1000) + REBOOT_DELAY_MS;

    // Update state
    setup_trigger_set_state(PROV_STATE_EXITING_SETUP, (uint32_t)(esp_timer_get_time() / 1000));

    // Notify LCD
    if (s_config.lcd_callback) {
        s_status.state = PROV_STATE_EXITING_SETUP;
        s_config.lcd_callback(&s_status);
    }
}

/**
 * @brief Start setup mode (SoftAP + Web Server)
 */
static esp_err_t start_setup_mode(void)
{
    ESP_LOGI(TAG, "Starting setup mode...");

    // Start SoftAP
    esp_err_t ret = setup_softap_start(s_status.softap_ssid);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start SoftAP: %s", esp_err_to_name(ret));
        return ret;
    }

    // Copy IP address
    strncpy(s_status.softap_ip, setup_softap_get_ip(), sizeof(s_status.softap_ip) - 1);

    // Start DNS captive portal
    ret = setup_softap_start_dns();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to start DNS server: %s", esp_err_to_name(ret));
        // Continue anyway - DNS is not critical
    }

    // Start web server
    ret = setup_webserver_start(on_config_saved);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start web server: %s", esp_err_to_name(ret));
        setup_softap_stop();
        return ret;
    }

    ESP_LOGI(TAG, "Setup mode started: SSID=%s, IP=%s",
             s_status.softap_ssid, s_status.softap_ip);

    return ESP_OK;
}

/**
 * @brief Stop setup mode
 */
static void stop_setup_mode(void)
{
    ESP_LOGI(TAG, "Stopping setup mode...");

    setup_webserver_stop();
    setup_softap_stop();

    ESP_LOGI(TAG, "Setup mode stopped");
}

esp_err_t provisioning_mgr_init(const prov_config_t *config)
{
    if (s_initialized) {
        return ESP_OK;
    }

    if (config == NULL) {
        ESP_LOGE(TAG, "Config is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    // Copy configuration
    memcpy(&s_config, config, sizeof(prov_config_t));

    // Initialize sub-components
    setup_trigger_init();

    // Initialize status
    memset(&s_status, 0, sizeof(s_status));
    s_status.state = PROV_STATE_NORMAL;

    s_initialized = true;
    s_reboot_pending = false;

    ESP_LOGI(TAG, "Provisioning manager initialized");
    return ESP_OK;
}

bool provisioning_mgr_process_key(char key, bool pressed)
{
    if (!s_initialized) {
        return false;
    }

    uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
    prov_state_t old_state = setup_trigger_get_state();

    bool consumed = setup_trigger_process_key(key, pressed, now_ms);

    prov_state_t new_state = setup_trigger_get_state();

    // Handle state transitions that require action
    if (new_state != old_state) {
        ESP_LOGI(TAG, "State transition: %d -> %d", old_state, new_state);

        switch (new_state) {
            case PROV_STATE_NORMAL:
                if (old_state == PROV_STATE_SETUP_ACTIVE) {
                    stop_setup_mode();
                    s_reboot_pending = false;
                }
                break;

            case PROV_STATE_ENTERING_SETUP:
                // Start setup mode
                if (start_setup_mode() == ESP_OK) {
                    setup_trigger_set_state(PROV_STATE_SETUP_ACTIVE, now_ms);
                } else {
                    // Failed to start setup, return to normal
                    setup_trigger_reset();
                }
                break;

            case PROV_STATE_EXITING_SETUP:
                // Stop setup mode and schedule reboot
                stop_setup_mode();
                s_reboot_pending = true;
                s_reboot_time_ms = now_ms + REBOOT_DELAY_MS;
                setup_trigger_set_state(PROV_STATE_REBOOTING, now_ms);
                break;

            default:
                break;
        }

        // Notify LCD callback
        update_status_and_notify(now_ms, false);
    }

    return consumed;
}

void provisioning_mgr_update(uint32_t now_ms)
{
    if (!s_initialized) {
        return;
    }

    prov_state_t old_state = setup_trigger_get_state();

    // Update trigger state machine
    setup_trigger_update(now_ms);

    prov_state_t new_state = setup_trigger_get_state();

    // Handle state transitions
    if (new_state != old_state) {
        ESP_LOGI(TAG, "State transition (update): %d -> %d", old_state, new_state);

        switch (new_state) {
            case PROV_STATE_NORMAL:
                if (old_state == PROV_STATE_SETUP_ACTIVE) {
                    stop_setup_mode();
                    s_reboot_pending = false;
                }
                break;

            case PROV_STATE_ENTERING_SETUP:
                // Start setup mode
                if (start_setup_mode() == ESP_OK) {
                    setup_trigger_set_state(PROV_STATE_SETUP_ACTIVE, now_ms);
                    new_state = PROV_STATE_SETUP_ACTIVE;
                } else {
                    setup_trigger_reset();
                    new_state = PROV_STATE_NORMAL;
                }
                break;

            case PROV_STATE_EXITING_SETUP:
                // Stop setup mode and schedule reboot
                stop_setup_mode();
                s_reboot_pending = true;
                s_reboot_time_ms = now_ms + REBOOT_DELAY_MS;
                setup_trigger_set_state(PROV_STATE_REBOOTING, now_ms);
                new_state = PROV_STATE_REBOOTING;
                break;

            default:
                break;
        }

        // Notify LCD callback
        update_status_and_notify(now_ms, false);
    }

    // Check for pending reboot
    if (s_reboot_pending && now_ms >= s_reboot_time_ms) {
        ESP_LOGI(TAG, "Rebooting now...");
        vTaskDelay(pdMS_TO_TICKS(100));  // Small delay for logs to flush
        esp_restart();
    }

    // Periodic status update for hold progress display
    static uint32_t last_status_update = 0;
    if (new_state == PROV_STATE_HASH_HOLDING ||
        new_state == PROV_STATE_PASSWORD_ENTRY ||
        new_state == PROV_STATE_SETUP_ACTIVE) {
        if ((now_ms - last_status_update) >= 200) {
            last_status_update = now_ms;
            update_status_and_notify(now_ms, true);  // force=true for periodic LCD updates
        }
    }
}

prov_state_t provisioning_mgr_get_state(void)
{
    if (!s_initialized) {
        return PROV_STATE_NORMAL;
    }
    return setup_trigger_get_state();
}

void provisioning_mgr_get_status(prov_status_t *status)
{
    if (status == NULL) {
        return;
    }
    memcpy(status, &s_status, sizeof(prov_status_t));
}

bool provisioning_mgr_is_setup_mode(void)
{
    if (!s_initialized) {
        return false;
    }
    prov_state_t state = setup_trigger_get_state();
    return (state == PROV_STATE_SETUP_ACTIVE ||
            state == PROV_STATE_ENTERING_SETUP ||
            state == PROV_STATE_EXITING_SETUP ||
            state == PROV_STATE_REBOOTING);
}

bool provisioning_mgr_is_normal_mode(void)
{
    if (!s_initialized) {
        return true;
    }
    prov_state_t state = setup_trigger_get_state();
    return (state == PROV_STATE_NORMAL);
}

esp_err_t provisioning_mgr_enter_setup(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
    prov_state_t state = setup_trigger_get_state();
    if (state == PROV_STATE_SETUP_ACTIVE) {
        update_status_and_notify(now_ms, true);
        return ESP_OK;
    }
    if (state != PROV_STATE_NORMAL) {
        return ESP_ERR_INVALID_STATE;
    }

    setup_trigger_set_state(PROV_STATE_ENTERING_SETUP, now_ms);
    update_status_and_notify(now_ms, true);

    esp_err_t ret = start_setup_mode();
    if (ret == ESP_OK) {
        setup_trigger_set_state(PROV_STATE_SETUP_ACTIVE, now_ms);
    } else {
        setup_trigger_reset();
    }
    update_status_and_notify(now_ms, true);
    return ret;
}

void provisioning_mgr_exit_setup(void)
{
    if (!s_initialized) {
        return;
    }

    prov_state_t state = setup_trigger_get_state();
    if (state == PROV_STATE_SETUP_ACTIVE) {
        uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
        setup_trigger_set_state(PROV_STATE_EXITING_SETUP, now_ms);

        // Stop setup mode
        stop_setup_mode();

        // Schedule reboot
        s_reboot_pending = true;
        s_reboot_time_ms = now_ms + REBOOT_DELAY_MS;

        // Update to rebooting state
        setup_trigger_set_state(PROV_STATE_REBOOTING, now_ms);
        update_status_and_notify(now_ms, false);
    }
}

void provisioning_mgr_request_reboot(void)
{
    uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
    s_reboot_pending = true;
    s_reboot_time_ms = now_ms + REBOOT_DELAY_MS;

    setup_trigger_set_state(PROV_STATE_REBOOTING, now_ms);
    update_status_and_notify(now_ms, false);
}
