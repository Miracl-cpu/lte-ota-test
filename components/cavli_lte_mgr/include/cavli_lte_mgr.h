/**
 * @file cavli_lte_mgr.h
 * @brief Cavli C16qs LTE module manager
 *
 * Manages the Cavli C16qs cellular module as a secondary uplink via UART2
 * AT commands. Provides the same MQTT publish interface as mqtt_client_mgr
 * so that json_data_logger callbacks work transparently in LTE mode.
 *
 * Build is incremental — API grows with each implementation block:
 *   Block 1 : UART init, AT send/receive, diagnostics
 *   Block 2 : Network registration (cavli_connect_once)
 *   Block 3 : MQTT connect + publish
 *   Block 4 : UART RX task + URC handling
 *   Block 5 : main.c integration
 *   Block 6 : Reconnect task + shutdown
 */

#ifndef CAVLI_LTE_MGR_H
#define CAVLI_LTE_MGR_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * Compile-time constants
 * ========================================================================= */

#define CAVLI_UART_NUM            2       // UART_NUM_2
#define CAVLI_UART_TX_GPIO        41      // ESP32-S3 GPIO41 → Cavli RX
#define CAVLI_UART_RX_GPIO        40      // ESP32-S3 GPIO40 → Cavli TX
#define CAVLI_UART_BAUD_RATE      115200
#define CAVLI_UART_RX_BUF_SIZE    2048

#define CAVLI_AT_TIMEOUT_MS       3000    // Per-command response timeout (ms)
#define CAVLI_CONN_TIMEOUT_MS     30000   // MQTT connect timeout (ms)
#define CAVLI_NET_REG_TIMEOUT_MS  60000   // Network registration timeout (ms)
#define CAVLI_MAX_PAYLOAD_SIZE    3500    // Safe limit below 4 KB modem default

// Keepalive loaded from NVS (device_cfg.mqtt_keepalive) — no compile constant

/* ============================================================================
 * Diagnostic types
 * ========================================================================= */

/**
 * @brief Cavli init/operation stage — updated at every step of cavli_connect_once().
 * Frozen at the failing step on error; used by future LCD page and UART logs.
 */
typedef enum {
    CAVLI_STATE_IDLE = 0,
    CAVLI_STATE_AT_LIVENESS,        // Step 1/3 : AT liveness check
    CAVLI_STATE_HW_RESET,           // Step 2   : AT+TRB
    CAVLI_STATE_ECHO_OFF,           // Step 4   : ATE0
    CAVLI_STATE_ERROR_REPORTING,    // Step 5   : AT+CMEE=2
    CAVLI_STATE_RADIO_ON,           // Step 6   : AT+CFUN=1
    CAVLI_STATE_SIM_CHECK,          // Step 7   : AT+CPIN?
    CAVLI_STATE_NET_REGISTRATION,   // Step 8   : AT+CEREG? (poll)
    CAVLI_STATE_SIGNAL_CHECK,       // Step 9   : AT+CSQ
    CAVLI_STATE_PDP_CONTEXT,        // Step 10  : AT+CGDCONT
    CAVLI_STATE_PDP_ACTIVATE,       // Step 11  : AT+CGACT
    CAVLI_STATE_MQTT_CREATE,        // Step 12  : AT+MQTTCREATE
    CAVLI_STATE_MQTT_CONNECT,       // Step 13  : AT+MQTTCONN
    CAVLI_STATE_MQTT_BIRTH,         // Step 14  : publish "online"
    CAVLI_STATE_MQTT_SUBSCRIBE,     // Step 15  : AT+MQTTSUBUNSUB
    CAVLI_STATE_CONNECTED,          // Fully operational
    CAVLI_STATE_DISCONNECTED,       // Was connected — Tier 1 auto-reconnect active
    CAVLI_STATE_RECONNECTING,       // Tier 2 background task retrying full init
} cavli_state_t;

/**
 * @brief Cavli error codes — set on abort; CAVLI_ERR_NONE when healthy.
 */
typedef enum {
    CAVLI_ERR_NONE = 0,
    CAVLI_ERR_AT_NO_RESPONSE,       // Modem not responding to AT
    CAVLI_ERR_SIM_NOT_READY,        // +CPIN not READY
    CAVLI_ERR_NET_REG_TIMEOUT,      // AT+CEREG polling timed out
    CAVLI_ERR_PDP_FAILED,           // AT+CGACT failed
    CAVLI_ERR_MQTT_CREATE_FAILED,   // AT+MQTTCREATE did not return token
    CAVLI_ERR_MQTT_CONNECT_TIMEOUT, // AT+MQTTCONN no CONNECTED within timeout
    CAVLI_ERR_PUBLISH_FAILED,       // AT+MQTTPUBLM PUBLISH FAIL or > timeout
    CAVLI_ERR_PUBLISH_TIMEOUT,      // No PUBLISH SUCCESS in time
} cavli_error_t;

/**
 * @brief Combined health + diagnostic struct (replaces cavli_status_t).
 * Read-safe via cavli_lte_mgr_get_diag() which takes g_diag_mutex internally.
 */
typedef struct {
    cavli_state_t  state;            // Current stage or stage at last failure
    cavli_error_t  last_error;       // CAVLI_ERR_NONE if healthy
    char           error_detail[64]; // +CME ERROR string (from AT+CMEE=2)
    bool           connected;        // Quick access: true = CAVLI_STATE_CONNECTED
    bool           sim_ready;        // AT+CPIN? == READY
    int8_t         signal_rssi;      // Last AT+CSQ value (0-31; 99 = unknown)
    int16_t        rssi_dbm;         // Derived from CSQ; INT16_MIN = unknown
    int16_t        rsrp_dbm;         // Derived from AT+CESQ; INT16_MIN = unknown
    int16_t        rsrq_db;          // Derived from AT+CESQ; INT16_MIN = unknown
    int16_t        mqtt_reason_code; // 0 when connected, modem reason if available
    char           sim_phone[20];    // SIM MSISDN from AT+CNUM ("" if not stored on SIM)
    char           operator_name[24];
    char           apn[32];
    char           ip_addr[32];
    char           band[16];
    uint32_t       messages_sent;
    uint32_t       messages_failed;
    uint32_t       connect_attempts; // Total Tier 2 reconnect attempts since boot
    uint32_t       consecutive_publish_failures;
    uint32_t       mqtt_reconnects;
    uint32_t       modem_restarts;
    char           last_recovery_action[24];
} cavli_diag_t;

/* ============================================================================
 * Public API — Block 1 (UART + AT communication)
 * ========================================================================= */

/**
 * @brief Store MQTT/broker parameters and build topic strings.
 * Creates g_uart_mutex and g_diag_mutex.
 * Must be called before cavli_lte_mgr_start().
 *
 * @param broker_host  MQTT broker hostname or IP
 * @param broker_port  MQTT broker port
 * @param client_id    MQTT client ID (device_id from NVS)
 * @param username     MQTT username
 * @param password     MQTT password
 * @param base_topic   Base topic string e.g. "Limelight/factory/DM-004"
 * @param keepalive    MQTT keepalive in seconds (from device_cfg.mqtt_keepalive)
 * @return ESP_OK on success
 */
esp_err_t cavli_lte_mgr_init(const char *broker_host, uint16_t broker_port,
                               const char *client_id,   const char *username,
                               const char *password,    const char *base_topic,
                               uint16_t keepalive);

/**
 * @brief Initialise UART2, start UART RX task, and launch cavli_reconnect_task.
 * The reconnect task performs the initial connect attempt immediately, then
 * retries in the background with exponential backoff (60 s → 120 s → 300 s cap).
 * Always returns ESP_OK — app_main continues regardless of connectivity state.
 * @return ESP_OK
 */
esp_err_t cavli_lte_mgr_start(void);

/**
 * @brief Gracefully stop all Cavli background tasks and release UART2.
 *
 * Sets g_stop_requested, waits for both tasks to exit, sends AT+MQTTDISCONN,
 * blind-blasts AT+MQTTDELETE for tokens 3-5, deinits UART2, deletes mutexes,
 * and resets all internal state. After this call,
 * cavli_lte_mgr_init() + cavli_lte_mgr_start() may be called again.
 *
 * Blocking: may wait up to ~90 s if the reconnect task is mid-connect-sequence.
 */
void cavli_lte_mgr_stop(void);

/**
 * @brief Request an immediate Cavli modem reboot with AT+TRB.
 *
 * The command uses the manager's UART mutex, so it cannot overlap an MQTT or
 * diagnostic AT exchange. The manager must already be running.
 */
esp_err_t cavli_lte_mgr_restart_modem(void);

/**
 * @brief Thread-safe check for MQTT connectivity.
 * @return true if g_diag.connected == true
 */
bool cavli_lte_mgr_is_connected(void);

/**
 * @brief Copy current diagnostics under g_diag_mutex.
 * Safe to call from any task.
 * @param out  Output buffer; must not be NULL
 */
void cavli_lte_mgr_get_diag(cavli_diag_t *out);

/**
 * @brief Trigger a background refresh of LTE diagnostic fields using AT commands.
 * Safe to call repeatedly; returns ESP_OK if a refresh is already in progress.
 */
esp_err_t cavli_lte_mgr_refresh_diag_async(void);

/**
 * @brief Convert state enum to human-readable string.
 * @return "NET_REGISTRATION", "CONNECTED", etc.
 */
const char *cavli_state_to_str(cavli_state_t s);

/**
 * @brief Convert error enum to human-readable string.
 * @return "NET_REG_TIMEOUT", "NONE", etc.
 */
const char *cavli_error_to_str(cavli_error_t e);

/* ============================================================================
 * Public API — Block 3 (MQTT publish + config callback)
 * ========================================================================= */

/**
 * @brief Callback type for incoming config messages received from the broker.
 * Invoked by the UART RX task (Block 4) when a +MQTTPUBLISH URC arrives on
 * the config topic. Same signature as mqtt_config_cb_t in mqtt_client_mgr.h.
 */
typedef void (*cavli_config_cb_t)(const char *config_json);

/**
 * @brief Publish a message to the broker using AT+MQTTPUBLM.
 * Thread-safe: acquires g_uart_mutex for the full two-step exchange
 * (command → '>' prompt → payload bytes → PUBLISH SUCCESS).
 * @param topic    Full MQTT topic string
 * @param payload  Null-terminated payload string
 * @param qos      QoS level (0 or 1)
 * @return ESP_OK on PUBLISH SUCCESS; ESP_FAIL if not connected or publish failed
 */
esp_err_t cavli_lte_mgr_publish(const char *topic, const char *payload, int qos);

/**
 * @brief Register callback for incoming config messages on the config topic.
 * @param cb  Callback function; NULL to deregister
 */
void cavli_lte_mgr_set_config_callback(cavli_config_cb_t cb);

/**
 * @brief Publish a config acknowledgement to <base_topic>/config/ack (QoS 1).
 * Payload schema matches mqtt_client_mgr_publish_config_ack() exactly:
 *   { "msg_type":"config.ack", "config_id":..., "catalog_version":...,
 *     "status":..., "ts":"<ISO8601>", "error":... }
 * @param config_id        Config message ID from the incoming JSON
 * @param catalog_version  Catalog version from the incoming JSON
 * @param status           "success" or "error"
 * @param error            Error detail (NULL or "" → field omitted from JSON)
 * @return ESP_OK on success; ESP_ERR_INVALID_STATE if disconnected
 */
esp_err_t cavli_lte_mgr_publish_config_ack(const char *config_id,
                                             const char *catalog_version,
                                             const char *status,
                                             const char *error);

#ifdef __cplusplus
}
#endif

#endif // CAVLI_LTE_MGR_H
