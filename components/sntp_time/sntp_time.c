/**
 * @file sntp_time.c
 * @brief SNTP Time Synchronization implementation for CT+HMI System
 *
 * Uses ESP-IDF SNTP component for NTP time synchronization.
 * Configures multiple NTP servers for reliability.
 */

#include "sntp_time.h"
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "log_config.h"
#undef LOG_LOCAL_LEVEL
#define LOG_LOCAL_LEVEL LOG_LEVEL_SNTP
#include "esp_log.h"
#include "esp_sntp.h"

static const char *TAG = "sntp_time";

// Module state
static struct {
    bool initialized;
    bool synced;
    char timezone[32];
} s_sntp = {0};

esp_err_t sntp_time_set_timezone(const char *timezone)
{
    if (timezone != NULL && strlen(timezone) > 0) {
        strncpy(s_sntp.timezone, timezone, sizeof(s_sntp.timezone) - 1);
        s_sntp.timezone[sizeof(s_sntp.timezone) - 1] = '\0';
    } else {
        strcpy(s_sntp.timezone, "UTC0");
    }

    setenv("TZ", s_sntp.timezone, 1);
    tzset();
    ESP_LOGI(TAG, "Timezone set: %s", s_sntp.timezone);
    return ESP_OK;
}

/**
 * @brief SNTP sync notification callback
 */
static void time_sync_notification_cb(struct timeval *tv)
{
    s_sntp.synced = true;

    time_t now = tv->tv_sec;
    struct tm timeinfo;
    localtime_r(&now, &timeinfo);

    char strftime_buf[64];
    strftime(strftime_buf, sizeof(strftime_buf), "%Y-%m-%dT%H:%M:%S", &timeinfo);
    ESP_LOGI(TAG, "Time synchronized: %s (TZ: %s)", strftime_buf, s_sntp.timezone);
}

esp_err_t sntp_time_init(const char *timezone)
{
    if (s_sntp.initialized) {
        ESP_LOGW(TAG, "SNTP already initialized");
        return ESP_OK;
    }

    esp_err_t ret = sntp_time_set_timezone(timezone);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set timezone: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "Initializing SNTP, timezone: %s", s_sntp.timezone);

    // Configure SNTP
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);

    // Set NTP servers (using indices 0 and 1)
    esp_sntp_setservername(0, "pool.ntp.org");
    esp_sntp_setservername(1, "time.google.com");

    // Set sync notification callback
    sntp_set_time_sync_notification_cb(time_sync_notification_cb);

    // Set sync mode to smooth (gradually adjust time)
    sntp_set_sync_mode(SNTP_SYNC_MODE_SMOOTH);

    // Initialize and start SNTP
    esp_sntp_init();

    s_sntp.initialized = true;
    s_sntp.synced = false;

    ESP_LOGI(TAG, "SNTP initialized, waiting for time sync...");

    return ESP_OK;
}

bool sntp_time_is_synced(void)
{
    if (s_sntp.initialized) {
        // Check SNTP sync status
        sntp_sync_status_t sync_status = sntp_get_sync_status();

        if (sync_status == SNTP_SYNC_STATUS_COMPLETED) {
            s_sntp.synced = true;
            return true;
        }
    }

    // Also accept a valid system clock set by LTE modem time (AT+CCLK).
    time_t now;
    struct tm timeinfo;
    time(&now);
    localtime_r(&now, &timeinfo);

    if (timeinfo.tm_year > (2020 - 1900)) {
        s_sntp.synced = true;
        return true;
    }

    return false;
}

bool sntp_time_wait_for_sync(uint32_t timeout_ms)
{
    if (!s_sntp.initialized) {
        ESP_LOGE(TAG, "SNTP not initialized");
        return false;
    }

    uint32_t elapsed = 0;
    const uint32_t poll_interval_ms = 100;

    ESP_LOGI(TAG, "Waiting for NTP sync (timeout: %lums)...", (unsigned long)timeout_ms);

    while (elapsed < timeout_ms) {
        if (sntp_time_is_synced()) {
            ESP_LOGI(TAG, "NTP sync completed after %lums", (unsigned long)elapsed);
            return true;
        }

        vTaskDelay(pdMS_TO_TICKS(poll_interval_ms));
        elapsed += poll_interval_ms;
    }

    ESP_LOGW(TAG, "NTP sync timeout after %lums", (unsigned long)timeout_ms);
    return false;
}
