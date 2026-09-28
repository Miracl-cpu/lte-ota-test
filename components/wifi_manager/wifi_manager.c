/**
 * @file wifi_manager.c
 * @brief WiFi Manager implementation for CT+HMI System
 *
 * Uses ESP-IDF WiFi station mode with event-driven connection management.
 * Features auto-reconnect on disconnect and periodic RSSI updates.
 *
 * Fix history:
 * - Removed vTaskDelay from WIFI_EVENT_STA_DISCONNECTED handler (#1).
 *   Reconnect is now scheduled via esp_timer one-shot, so the default
 *   event loop task is never blocked.
 * - Added exponential backoff with jitter on reconnect (#12).
 * - Changed auth threshold to WIFI_AUTH_WPA_PSK to accept WPA/WPA2/WPA3 (#9).
 * - Removed nvs_flash_init() — NVS is initialised before wifi_manager_init()
 *   is called; a second init risks erasing provisioned config (#16).
 */

#include "wifi_manager.h"
#include "nvs_config.h"
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_timer.h"
#include "log_config.h"
#undef LOG_LOCAL_LEVEL
#define LOG_LOCAL_LEVEL LOG_LEVEL_WIFI
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_random.h"

static const char *TAG = "wifi_manager";

// Event group bits
#define WIFI_CONNECTED_BIT  BIT0
#define WIFI_FAIL_BIT       BIT1

// Default retry interval if not specified
#define DEFAULT_RETRY_INTERVAL_MS   5000

// RSSI update interval
#define RSSI_UPDATE_INTERVAL_MS     5000

// ESP-IDF uses quarter-dBm units. 52 = 13 dBm.
// This trims WiFi current spikes on marginal standalone supplies while keeping
// enough range for normal factory-floor AP distances.
#define WIFI_MAX_TX_POWER_QDBM      52

// Exponential backoff parameters (#12)
#define BACKOFF_BASE_MS    1000   // 1s base
#define BACKOFF_MAX_MS     30000  // 30s cap
#define BACKOFF_JITTER_MS  1000   // 0-999ms random jitter

// Module state
static struct {
    wifi_manager_config_t config;
    wifi_status_t status;
    EventGroupHandle_t event_group;
    esp_netif_t *sta_netif;
    wifi_connected_cb_t connected_cb;
    esp_timer_handle_t reconnect_timer;  // one-shot timer for reconnect (#1)
    SemaphoreHandle_t status_mutex;      // protects s_wifi.status (NEW-5)
    bool initialized;
    bool started;
    int retry_count;
    uint32_t last_rssi_update_ms;  // throttle for inline RSSI refresh (#15)
} s_wifi = {0};

/**
 * @brief Compute reconnect delay with exponential backoff + jitter (#12)
 *
 * Sequence: 1s, 2s, 4s, 8s, 16s, 30s (capped), all +0..999ms jitter.
 */
static uint32_t get_backoff_ms(int retry_count)
{
    uint32_t exp_ms = BACKOFF_BASE_MS;
    for (int i = 0; i < retry_count && exp_ms < BACKOFF_MAX_MS; i++) {
        exp_ms *= 2;
    }
    if (exp_ms > BACKOFF_MAX_MS) {
        exp_ms = BACKOFF_MAX_MS;
    }
    return exp_ms + (esp_random() % BACKOFF_JITTER_MS);
}

/**
 * @brief esp_timer callback — fires outside the event loop task (#1)
 *
 * Called by the high-priority timer task after the backoff delay.
 * Safe to call esp_wifi_connect() here.
 */
static const char *wifi_disconnect_reason_name(uint8_t reason)
{
    switch (reason) {
        case WIFI_REASON_AUTH_EXPIRE:              return "auth_expire";
        case WIFI_REASON_ASSOC_LEAVE:              return "assoc_leave";
        case WIFI_REASON_4WAY_HANDSHAKE_TIMEOUT:   return "4way_timeout";
        case WIFI_REASON_GROUP_KEY_UPDATE_TIMEOUT: return "group_key_timeout";
        case WIFI_REASON_BEACON_TIMEOUT:           return "beacon_timeout";
        case WIFI_REASON_NO_AP_FOUND:              return "no_ap_found";
        case WIFI_REASON_AUTH_FAIL:                return "auth_fail";
        case WIFI_REASON_ASSOC_FAIL:               return "assoc_fail";
        case WIFI_REASON_HANDSHAKE_TIMEOUT:        return "handshake_timeout";
        case WIFI_REASON_CONNECTION_FAIL:          return "connection_fail";
        case WIFI_REASON_AP_TSF_RESET:             return "ap_tsf_reset";
        case WIFI_REASON_ROAMING:                  return "roaming";
        default:                                   return "unknown";
    }
}

static void reconnect_timer_cb(void *arg)
{
    ESP_LOGI(TAG, "Reconnect timer fired, calling esp_wifi_connect() (retry=%d)",
             s_wifi.retry_count);
    esp_err_t ret = esp_wifi_connect();
    if (ret != ESP_OK && ret != ESP_ERR_WIFI_CONN) {
        ESP_LOGW(TAG, "esp_wifi_connect failed from reconnect timer: %s",
                 esp_err_to_name(ret));
    }
}

/**
 * @brief WiFi event handler
 */
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                                int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT) {
        switch (event_id) {
            case WIFI_EVENT_STA_START:
                ESP_LOGI(TAG, "WiFi station started, connecting...");
                esp_wifi_connect();
                break;

            case WIFI_EVENT_STA_DISCONNECTED: {
                wifi_event_sta_disconnected_t *event =
                    (wifi_event_sta_disconnected_t *)event_data;

                /* #ISSUE-42: use bounded timeout — portMAX_DELAY in the event loop
                 * task would stall ALL system events if mutex is ever contended. */
                if (xSemaphoreTake(s_wifi.status_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                    s_wifi.status.connected = false;
                    s_wifi.status.rssi = -100;
                    s_wifi.last_rssi_update_ms = 0;  // force fresh read on next connect (#15)
                    memset(s_wifi.status.ip_addr, 0, sizeof(s_wifi.status.ip_addr));
                    xSemaphoreGive(s_wifi.status_mutex);
                } else {
                    ESP_LOGW(TAG, "status_mutex timeout in disconnect handler — status not updated");
                }

                if (!s_wifi.started) {
                    ESP_LOGI(TAG, "WiFi disconnect event during stop/shutdown");
                    break;
                }

                uint32_t delay_ms = get_backoff_ms(s_wifi.retry_count);
                s_wifi.retry_count++;

                ESP_LOGW(TAG,
                         "WiFi disconnected (reason=%u:%s), retry #%d in %lums...",
                         (unsigned)event->reason,
                         wifi_disconnect_reason_name(event->reason),
                         s_wifi.retry_count,
                         (unsigned long)delay_ms);

                /*
                 * FIX #1: Schedule reconnect via one-shot timer instead of
                 * vTaskDelay().  vTaskDelay() inside the default event loop
                 * task blocks ALL system events (IP, MQTT, provisioning) for
                 * the full retry interval.  The timer callback runs in the
                 * esp_timer task, which is unrelated to the event loop.
                 *
                 * Stop first in case a previous timer is still pending
                 * (e.g., rapid disconnect-reconnect-disconnect).
                 */
                esp_timer_stop(s_wifi.reconnect_timer);  /* OK if not running */
                esp_timer_start_once(s_wifi.reconnect_timer,
                                     (uint64_t)delay_ms * 1000ULL);
                break;
            }

            case WIFI_EVENT_STA_CONNECTED:
                ESP_LOGI(TAG, "WiFi associated with AP, waiting for IP...");
                break;

            default:
                break;
        }
    }
}

/**
 * @brief IP event handler
 */
static void ip_event_handler(void *arg, esp_event_base_t event_base,
                              int32_t event_id, void *event_data)
{
    if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;

        // Update status under mutex
        wifi_ap_record_t ap_info;
        int8_t rssi = -100;
        if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
            rssi = ap_info.rssi;
        }
        /* #ISSUE-42: use bounded timeout — portMAX_DELAY in the event loop
         * task would stall ALL system events if mutex is ever contended. */
        if (xSemaphoreTake(s_wifi.status_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            s_wifi.status.connected = true;
            snprintf(s_wifi.status.ip_addr, sizeof(s_wifi.status.ip_addr),
                     IPSTR, IP2STR(&event->ip_info.ip));
            s_wifi.status.rssi = rssi;
            xSemaphoreGive(s_wifi.status_mutex);
        } else {
            ESP_LOGW(TAG, "status_mutex timeout in got_ip handler — status not updated");
        }
        s_wifi.retry_count = 0;   // reset backoff counter on successful connect

        ESP_LOGI(TAG, "Connected to WiFi, IP: %s, RSSI: %d dBm",
                 s_wifi.status.ip_addr, s_wifi.status.rssi);

        // Signal connected event
        if (s_wifi.event_group != NULL) {
            xEventGroupSetBits(s_wifi.event_group, WIFI_CONNECTED_BIT);
        }

        // Call user callback if registered
        if (s_wifi.connected_cb != NULL) {
            s_wifi.connected_cb();
        }
    }
}


esp_err_t wifi_manager_init(const wifi_manager_config_t *config)
{
    if (s_wifi.initialized) {
        ESP_LOGW(TAG, "WiFi manager already initialized");
        return ESP_OK;
    }

    // If config is NULL, load from NVS
    if (config == NULL) {
        nvs_device_config_t nvs_snap;
        const nvs_device_config_t *nvs_cfg = nvs_config_snapshot(&nvs_snap) ? &nvs_snap : NULL;
        if (nvs_cfg == NULL || !nvs_cfg->wifi_configured) {
            ESP_LOGE(TAG, "No WiFi configuration available in NVS");
            return ESP_ERR_NOT_FOUND;
        }
        strncpy(s_wifi.config.ssid, nvs_cfg->wifi_ssid, sizeof(s_wifi.config.ssid) - 1);
        strncpy(s_wifi.config.password, nvs_cfg->wifi_password, sizeof(s_wifi.config.password) - 1);
        s_wifi.config.retry_interval_ms = DEFAULT_RETRY_INTERVAL_MS;
        ESP_LOGI(TAG, "WiFi config loaded from NVS");
    } else {
        memcpy(&s_wifi.config, config, sizeof(wifi_manager_config_t));
    }

    if (s_wifi.config.retry_interval_ms == 0) {
        s_wifi.config.retry_interval_ms = DEFAULT_RETRY_INTERVAL_MS;
    }

    /*
     * FIX #16: Do NOT call nvs_flash_init() here.
     * NVS is guaranteed to be fully initialized by nvs_config_init() which
     * is called before wifi_manager_init() in app_main.  A second
     * nvs_flash_init() that triggers nvs_flash_erase() would silently wipe
     * all provisioned credentials.
     */

    // Create reconnect timer (one-shot, not running yet) — fix #1
    esp_timer_create_args_t timer_args = {
        .callback = reconnect_timer_cb,
        .arg      = NULL,
        .name     = "wifi_reconnect",
    };
    esp_err_t ret = esp_timer_create(&timer_args, &s_wifi.reconnect_timer);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create reconnect timer: %s", esp_err_to_name(ret));
        return ret;
    }

    // Create status mutex (NEW-5)
    s_wifi.status_mutex = xSemaphoreCreateMutex();
    if (s_wifi.status_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create status mutex");
        esp_timer_delete(s_wifi.reconnect_timer);
        s_wifi.reconnect_timer = NULL;
        return ESP_ERR_NO_MEM;
    }

    // Create event group
    s_wifi.event_group = xEventGroupCreate();
    if (s_wifi.event_group == NULL) {
        ESP_LOGE(TAG, "Failed to create event group");
        vSemaphoreDelete(s_wifi.status_mutex);
        s_wifi.status_mutex = NULL;
        esp_timer_delete(s_wifi.reconnect_timer);
        s_wifi.reconnect_timer = NULL;
        return ESP_ERR_NO_MEM;
    }

    // Initialize TCP/IP stack
    ret = esp_netif_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init netif: %s", esp_err_to_name(ret));
        return ret;
    }

    // Create default event loop
    ret = esp_event_loop_create_default();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "Failed to create event loop: %s", esp_err_to_name(ret));
        return ret;
    }

    // Create default WiFi station
    s_wifi.sta_netif = esp_netif_create_default_wifi_sta();
    if (s_wifi.sta_netif == NULL) {
        ESP_LOGE(TAG, "Failed to create default WiFi STA");
        return ESP_FAIL;
    }

    // Initialize WiFi with default config
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ret = esp_wifi_init(&cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init WiFi: %s", esp_err_to_name(ret));
        return ret;
    }

    // Register event handlers
    ret = esp_event_handler_instance_register(WIFI_EVENT,
                                               ESP_EVENT_ANY_ID,
                                               &wifi_event_handler,
                                               NULL, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register WiFi event handler: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = esp_event_handler_instance_register(IP_EVENT,
                                               IP_EVENT_STA_GOT_IP,
                                               &ip_event_handler,
                                               NULL, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register IP event handler: %s", esp_err_to_name(ret));
        return ret;
    }

    // Configure WiFi station
    wifi_config_t wifi_config = {0};
    strncpy((char *)wifi_config.sta.ssid, s_wifi.config.ssid, sizeof(wifi_config.sta.ssid) - 1);
    strncpy((char *)wifi_config.sta.password, s_wifi.config.password, sizeof(wifi_config.sta.password) - 1);
    /*
     * FIX #9: Use WIFI_AUTH_WPA_PSK (minimum threshold) instead of
     * WIFI_AUTH_WPA2_PSK.  This allows connecting to WPA, WPA2, and WPA3
     * access points.  WIFI_AUTH_WPA2_PSK silently rejects WPA3-only APs.
     */
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA_PSK;
    wifi_config.sta.pmf_cfg.capable = true;
    wifi_config.sta.pmf_cfg.required = false;

    ret = esp_wifi_set_mode(WIFI_MODE_STA);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set WiFi mode: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set WiFi config: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = esp_wifi_set_ps(WIFI_PS_NONE);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to disable WiFi power save: %s",
                 esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "WiFi power save disabled for stable STA connection");
    }

    ret = esp_wifi_set_max_tx_power(WIFI_MAX_TX_POWER_QDBM);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to set WiFi TX power: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "WiFi TX power capped at %.2f dBm",
                 WIFI_MAX_TX_POWER_QDBM / 4.0f);
    }

    // Initialize status
    s_wifi.status.connected = false;
    s_wifi.status.rssi = -100;
    memset(s_wifi.status.ip_addr, 0, sizeof(s_wifi.status.ip_addr));

    s_wifi.initialized = true;

    ESP_LOGI(TAG, "WiFi manager initialized, SSID: %s", s_wifi.config.ssid);

    return ESP_OK;
}

void wifi_manager_set_connected_callback(wifi_connected_cb_t cb)
{
    s_wifi.connected_cb = cb;
}

esp_err_t wifi_manager_start(void)
{
    if (!s_wifi.initialized) {
        ESP_LOGE(TAG, "WiFi manager not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (s_wifi.started) {
        ESP_LOGW(TAG, "WiFi already started");
        return ESP_OK;
    }

    esp_err_t ret = esp_wifi_start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start WiFi: %s", esp_err_to_name(ret));
        return ret;
    }

    s_wifi.started = true;

    ESP_LOGI(TAG, "WiFi started, connecting to %s...", s_wifi.config.ssid);

    return ESP_OK;
}

void wifi_manager_stop(void)
{
    if (!s_wifi.initialized) {
        return;
    }

    s_wifi.started = false;
    s_wifi.retry_count = 0;
    s_wifi.last_rssi_update_ms = 0;

    if (s_wifi.reconnect_timer != NULL) {
        esp_timer_stop(s_wifi.reconnect_timer);
    }

    esp_err_t ret = esp_wifi_disconnect();
    if (ret != ESP_OK && ret != ESP_ERR_WIFI_NOT_INIT &&
        ret != ESP_ERR_WIFI_NOT_STARTED && ret != ESP_ERR_WIFI_NOT_CONNECT) {
        ESP_LOGW(TAG, "esp_wifi_disconnect failed: %s", esp_err_to_name(ret));
    }

    ret = esp_wifi_stop();
    if (ret != ESP_OK && ret != ESP_ERR_WIFI_NOT_INIT &&
        ret != ESP_ERR_WIFI_NOT_STARTED) {
        ESP_LOGW(TAG, "esp_wifi_stop failed: %s", esp_err_to_name(ret));
    }

    if (s_wifi.status_mutex != NULL &&
        xSemaphoreTake(s_wifi.status_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        s_wifi.status.connected = false;
        s_wifi.status.rssi = -100;
        memset(s_wifi.status.ip_addr, 0, sizeof(s_wifi.status.ip_addr));
        xSemaphoreGive(s_wifi.status_mutex);
    }

    if (s_wifi.event_group != NULL) {
        xEventGroupClearBits(s_wifi.event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT);
    }

    ESP_LOGI(TAG, "WiFi stopped");
}

void wifi_manager_get_status(wifi_status_t *status)
{
    if (status == NULL) return;
    if (s_wifi.status_mutex == NULL) {          // Not initialized (offline mode)
        memset(status, 0, sizeof(wifi_status_t));
        return;
    }
    /* #ISSUE-47: bounded timeout — portMAX_DELAY can stall the calling task indefinitely.
     * Pattern: same as wifi_manager.c:116,177 fixed in #ISSUE-42. */
    if (xSemaphoreTake(s_wifi.status_mutex, pdMS_TO_TICKS(50)) != pdTRUE) {
        ESP_LOGW(TAG, "status_mutex timeout in get_status — returning zeroed struct");
        memset(status, 0, sizeof(wifi_status_t));
        return;
    }
    memcpy(status, &s_wifi.status, sizeof(wifi_status_t));
    xSemaphoreGive(s_wifi.status_mutex);
}

bool wifi_manager_is_connected(void)
{
    if (s_wifi.status_mutex == NULL) return false;  // Not initialized (offline mode)
    /* #ISSUE-47: bounded timeout — same policy as #ISSUE-42/#ISSUE-46. */
    if (xSemaphoreTake(s_wifi.status_mutex, pdMS_TO_TICKS(50)) != pdTRUE) {
        ESP_LOGW(TAG, "status_mutex timeout in is_connected — returning false");
        return false;
    }
    bool connected = s_wifi.status.connected;
    xSemaphoreGive(s_wifi.status_mutex);
    return connected;
}

int8_t wifi_manager_get_rssi(void)
{
    if (s_wifi.status_mutex == NULL) return -100;   // Not initialized (offline mode)

    if (xSemaphoreTake(s_wifi.status_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGW(TAG, "status_mutex timeout in get_rssi - returning -100");
        return -100;
    }
    bool connected = s_wifi.status.connected;
    xSemaphoreGive(s_wifi.status_mutex);

    if (!connected) return -100;

    /*
     * FIX #15: Throttled inline refresh — replaces the removed rssi_update_task.
     * Runs in the caller's task (HMI), which is already WDT-monitored, so any
     * hang in esp_wifi_sta_get_ap_info() is caught by the HMI watchdog.
     * last_rssi_update_ms is only written from the HMI task — no mutex needed.
     */
    uint32_t now_ms = pdTICKS_TO_MS(xTaskGetTickCount());
    if ((uint32_t)(now_ms - s_wifi.last_rssi_update_ms) >= RSSI_UPDATE_INTERVAL_MS) {
        wifi_ap_record_t ap_info;
        if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
            if (xSemaphoreTake(s_wifi.status_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                s_wifi.status.rssi = ap_info.rssi;
                xSemaphoreGive(s_wifi.status_mutex);
            }
        }
        s_wifi.last_rssi_update_ms = now_ms;  // stamp even on failure — avoid tight retry
    }

    if (xSemaphoreTake(s_wifi.status_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGW(TAG, "status_mutex timeout in get_rssi - returning -100");
        return -100;
    }
    int8_t rssi = s_wifi.status.rssi;
    xSemaphoreGive(s_wifi.status_mutex);
    return rssi;
}
