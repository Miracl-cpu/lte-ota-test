/**
 * @file json_data_logger.h
 * @brief JSON Data Logger for CT+HMI System - Phase 1 (Serial Output)
 *
 * This component generates JSON messages for:
 * - Machine state changes (ACT/IDL/STP)
 * - Stoppage reason codes
 * - Periodic live telemetry data
 *
 * JSON messages are output to serial log with MQTT topic format
 * for future Phase 2 WiFi/MQTT integration.
 */

#ifndef JSON_DATA_LOGGER_H
#define JSON_DATA_LOGGER_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Machine state enum (mirrors main.c states)
 */
typedef enum {
    JSON_STATE_ACT = 0,  // Active (machine running)
    JSON_STATE_IDL,      // Idle (awaiting reason code)
    JSON_STATE_STP       // Stopped (reason code provided)
} json_machine_state_t;

/**
 * @brief Configuration structure for JSON logger
 */
typedef struct {
    char client_id[16];            // Device ID e.g. "DM-002"
    uint32_t livedata_interval_ms; // Telemetry interval (default 5000ms)
    bool log_to_serial;            // Enable serial output
} json_logger_config_t;

/**
 * @brief Live data structure for telemetry
 */
typedef struct {
    float current_avg;             // Average current (Amps)
    float phase_current_r;         // CT Sensor phase R current [A]
    float phase_current_y;         // CT Sensor phase Y current [A]
    float phase_current_b;         // CT Sensor phase B current [A]
    float power_kw;                // Power in kW
    float power_factor;            // Power factor
    bool changeover_final;         // true for one delayed old-value publish
    bool proxy_changeover_final;   // true when proxy fields are old final values
    bool encoder_changeover_final; // true when encoder fields are old final values
    bool duration_changeover_final;// true when cycle time fields are old final values
    bool signal_count_changeover_final; // true when signal fields are old final values
    bool proxy_enabled;            // true when at least one proxy input is enabled
    uint8_t proxy_enabled_mask;     // Exclusive: 0x01=Port1/A, 0x02=Port2/B
    uint64_t count_total;          // Persistent common production count
    uint32_t count_total1;         // Proxy Port 1 / A (GPIO_NUM_15) pulse count
    uint32_t count_total2;         // Proxy Port 2 / B (GPIO_NUM_16) pulse count
    bool proxy_production_enabled; // true when proxy production page/MQTT is enabled
    bool proxy_meter_mode;         // true=meter/count, false=parts/count
    bool proxy_minute_ready;
    uint32_t proxy_count_last_60s;
    uint64_t proxy_minute_seq;
    uint32_t proxy_minute_boot_count;
    uint32_t proxy_capture_rejected, proxy_capture_overflow;
    uint8_t proxy_report_channel;
    float proxy_meter_per_count;   // Configured meter/count value [m]
    float proxy_cycle_time_s;      // Scrap cycle time [s]
    float proxy_total_length_m;    // Good-product length total [m]
    uint32_t proxy_scrap_count;    // Meter/count scrap count
    uint32_t proxy_good_count;     // Good products or simple parts count
    float proxy_last_pulse_s;      // Last completed HIGH duration [s]
    float proxy_total_length_a_m;  // Proxy A good-product length total [m]
    float proxy_total_length_b_m;  // Proxy B good-product length total [m]
    uint32_t proxy_scrap_count_a;  // Proxy A scrap count
    uint32_t proxy_scrap_count_b;  // Proxy B scrap count
    uint32_t proxy_good_count_a;   // Proxy A good count
    uint32_t proxy_good_count_b;   // Proxy B good count
    float proxy_last_pulse_a_s;    // Proxy A last completed HIGH duration [s]
    float proxy_last_pulse_b_s;    // Proxy B last completed HIGH duration [s]
    bool encoder_enabled;          // true when encoder input is enabled
    int32_t encoder_count;         // Raw quadrature count
    float encoder_rpm;             // Current encoder speed [RPM]
    float encoder_mpm;             // Current encoder line speed [m/min]
    float encoder_length_m;        // Integrated encoder length [m]
    bool encoder_scrap_mode;        // true when encoder scrap mode is active
    float encoder_diameter_mm;      // Encoder diameter setting [mm]
    float encoder_target_length_m;  // Target pipe length [m]
    float encoder_current_pipe_length_m; // Current pipe length [m]
    uint32_t encoder_good_count;    // Encoder scrap-mode good count
    uint32_t encoder_scrap_count;   // Encoder scrap-mode scrap count
    bool ct_sensor_enabled;         // Automize CT part-count feature active
    uint32_t ct_part_count;         // CT-threshold part count
    float ct_current_value;         // CT current used for part count [A]
    float ct_threshold;             // CT part-count threshold [A]
    uint32_t ct_stabilization_ms;   // CT part-count stabilization time [ms]
    bool duration_count_enabled;     // true when cycle time counting is enabled
    bool duration_selected_valid;    // true when a configured part is selected
    uint32_t duration_part_number;   // Active numeric part number
    uint32_t duration_cycle_time_s;  // Full product cycle time [s]
    uint32_t duration_operations_per_part; // Operations required per part
    uint32_t duration_operation_count; // Runtime operation count in current part
    float duration_threshold_a;      // Channel A operation count threshold [A]
    uint32_t duration_finished_count; // Finished parts counted by cycle time count
    bool signal_count_enabled[2];     // Signal A/B count enabled
    bool signal_selected_valid[2];    // Signal A/B active part valid
    uint32_t signal_part_number[2];   // Signal A/B part number
    uint32_t signal_cycle_time_s[2];  // Signal A/B cycle time [s]
    uint32_t signal_operations_per_part[2]; // Signal A/B ops per part
    uint32_t signal_operation_count[2]; // Signal A/B current operation progress
    uint32_t signal_finished_count[2]; // Signal A/B finished part count
    uint8_t signal_logic[2];          // 0=NO, 1=NC
    uint32_t signal_count_rearm_ms;   // Common signal rearm delay [ms]
    bool speed_enabled;              // true when Speed length mode is enabled
    float speed_current_length_m;    // Current above-threshold run length [m]
    float speed_total_length_m;      // Persisted cumulative speed length [m]
    float speed_mps;                 // Configured line speed [m/s]
    float speed_threshold_a;         // Channel A threshold for Speed [A]
    float today_kwh;                // Daily energy counter
    float total_kwh;                // Cumulative energy counter
    bool energy_reset_event;        // True for payload after a reset
    char energy_reset_reason[18];   // daily_reset or overflow_reset
    bool modbus_ct_enabled;         // true when AVF-133-M1 is active source
    bool modbus_ct_data_valid;      // true when latest Modbus CT data is fresh
    char modbus_ct_network[8];      // "3P-3W", "3P-4W", or "1P-2W"
    float modbus_ct_kva;            // Apparent power [kVA]
    float modbus_ct_vrn, modbus_ct_vyn, modbus_ct_vbn;
    float modbus_ct_vry, modbus_ct_vyb, modbus_ct_vbr;
    float modbus_ct_ir, modbus_ct_iy, modbus_ct_ib;
    float modbus_ct_i_avg;
    json_machine_state_t state;    // Current machine state
    // Vibration sensor (WTVB01-485) — populated when vib_data_valid == true
    bool vib_enabled;              // true when vibration sensor is enabled in settings
    bool  vib_data_valid;          // false if sensor not initialised or no samples yet
    float vib_vel_avg;             // Average velocity RMS over livedata window [mm/s]
    float vib_vel_peak;            // Peak velocity RMS over livedata window [mm/s]
    float vib_acc_avg;             // Average acceleration RMS [g]
    float vib_acc_peak;            // Peak acceleration RMS [g]
    float vib_temp;                // Latest sensor temperature [°C]
} json_livedata_t;

/**
 * @brief Health diagnostics data structure
 *
 * Published every 30s on topic: Limelight/factory/{client_id}/health
 */
typedef struct {
    // Uplink mode
    uint8_t  uplink_mode;        // 0=WiFi, 1=LTE (mirrors nvs_uplink_mode_t)
    // WiFi (populated when uplink_mode == 0)
    int8_t   wifi_rssi;
    char     wifi_ssid[32];
    bool     wifi_pass_set;      // true if WiFi password is non-empty
    bool     wifi_connected;
    // LTE (populated when uplink_mode == 1)
    int8_t   lte_rssi_csq;      // AT+CSQ value (0-31; 99=unknown)
    bool     lte_connected;
    char     lte_phone[20];      // SIM MSISDN from AT+CNUM ("" if not stored)
    char     lte_recovery_state[24];
    char     lte_last_error[24];
    uint32_t lte_consecutive_publish_failures;
    uint32_t lte_connect_attempts;
    uint32_t lte_mqtt_reconnects;
    uint32_t lte_modem_restarts;
    char     lte_last_recovery_action[24];
    // CT sensor
    float    ct_phase_a_vrms;    // Raw representative value (phase A Vrms)
    uint8_t  ct_open_mask;       // CT open flags: bit0=A, bit1=B, bit2=C
    // 3-phase currents (measured)
    float    ir, iy, ib;
    // Power (configured, not measured)
    float    voltage_nominal;
    float    power_factor;
    // Timing
    uint32_t hmi_max_step_ms, hmi_slow_steps;
    char hmi_slowest_stage[20];
    uint32_t lcd_completed_frames, lcd_slow_transfers, lcd_recoveries, lcd_last_transfer_ms;
    uint32_t uptime_sec;
    uint32_t boot_count;
    char     boot_reason[16];     // ESP-IDF reset reason for this boot
    // MQTT/data stats
    uint32_t msg_seq;
    uint32_t messages_sent;
    uint32_t messages_failed;
    // Memory
    uint32_t free_heap;
    uint32_t min_heap;
    // Errors
    uint32_t i2c_err_count;
    uint32_t ads_err_count;
    bool     relay_gpio_ok;  // #ISSUE-53: false if GPIO config failed at boot
    bool     lcd_ok;         // #ISSUE-54: false if lcd_init failed at boot
    bool     keypad_ok;      // #ISSUE-54: false if keypad_init failed at boot
    // Firmware
    char firmware_version[34];  // "v@" + up to 31 app-version characters + NUL
    char mac_id[18];             // WiFi STA MAC "XX:XX:XX:XX:XX:XX"
    // Device parameters (remote visibility)
    float    ct_gain[3];             // CT calibration factors [PhaseA, PhaseB, PhaseC]
    uint32_t machine_setup_time_ms;  // Machine setup time (ms)
    float    kw_threshold;           // Active power threshold (kW)
    float    kw_hysteresis;          // Hysteresis absolute value (kW)
    // Vibration sensor health
    bool     vib_init_ok;            // false if wtvb01_init() failed at boot
    bool     vib_data_valid;         // false if no successful reads yet
    char     vib_model[16];          // Sensor model string e.g. "WTVB01-485"
    float    vib_vel_rms;            // Latest velocity RMS snapshot [mm/s]
    float    vib_temp;               // Latest temperature snapshot [°C]
    uint32_t vib_err_count;          // Failures accumulated during current communication fault
    bool     vib_comm_error;         // Debounced current communication fault
    bool     modbus_ct_enabled;      // true if AVF-133-M1 source is enabled
    bool     modbus_ct_data_valid;   // latest Modbus CT snapshot is fresh
    uint32_t modbus_ct_err_count;    // cumulative communication errors
} json_health_t;

/**
 * @brief Initialize the JSON logger
 *
 * @param config Pointer to configuration structure
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t json_logger_init(const json_logger_config_t *config);

/**
 * @brief Log a machine state change event
 *
 * Outputs JSON: {"type":"state","client_id":"DM-002","state":"XXX"}
 * Topic: Limelight/factory/{client_id}/statechange
 *
 * @param new_state The new machine state
 */
void json_logger_state_change(json_machine_state_t new_state);

/**
 * @brief Log a stoppage reason code event
 *
 * Outputs JSON: {"type":"reason","client_id":"DM-002","reason_code":"XX",
 *                "reason_l1":"...", "reason_l2":"..."}
 * Topic: Limelight/factory/{client_id}/statechange
 *
 * @param reason_code The reason code (0-99)
 * @param l1 Category text from catalog (NULL if not available)
 * @param l2 Detail text from catalog (NULL if not available)
 */
void json_logger_reason_code(uint8_t reason_code, const char *l1,
                             const char *l2);

/**
 * @brief Log live telemetry data
 *
 * Outputs JSON with current, power, state, timestamp.
 * Topic: Limelight/factory/{client_id}/livedata
 *
 * @param data Pointer to livedata structure
 */
void json_logger_livedata(const json_livedata_t *data);

/**
 * @brief Check if periodic livedata should be sent
 *
 * Checks elapsed time since last livedata and sends if interval exceeded.
 * Call this periodically from HMI task.
 *
 * @param data Pointer to current livedata
 * @return true if livedata was sent, false otherwise
 */
bool json_logger_periodic_check(const json_livedata_t *data);

/**
 * @brief Send livedata immediately (bypasses interval check)
 *
 * Use this for state change events where immediate telemetry is needed.
 * Resets the periodic timer to prevent double-logging.
 *
 * @param data Pointer to current livedata
 */
void json_logger_livedata_immediate(const json_livedata_t *data);

/**
 * @brief MQTT publish callback function type
 *
 * @param topic Full MQTT topic string
 * @param payload JSON payload string
 * @return ESP_OK on successful publish, ESP_FAIL otherwise
 */
typedef esp_err_t (*mqtt_publish_fn_t)(const char *topic, const char *payload);

/**
 * @brief Set MQTT publish callbacks
 *
 * When callbacks are set, JSON output is sent via MQTT.
 * If MQTT publish fails or callbacks are NULL, falls back to serial logging.
 *
 * @param state_cb Callback for state/reason messages (QoS 1)
 * @param livedata_cb Callback for livedata messages (QoS 0)
 */
void json_logger_set_mqtt_callback(mqtt_publish_fn_t state_cb, mqtt_publish_fn_t livedata_cb);

/**
 * @brief Check if MQTT publishing is available
 *
 * @return true if MQTT callbacks are registered
 */
bool json_logger_mqtt_available(void);

/**
 * @brief Re-publish last known statechange and reason JSON after reconnect
 *
 * Call this from the MQTT CONNECTED event to ensure the cloud receives
 * the current machine state even if the original publish was lost during
 * a connectivity gap. Uses the same seq numbers so cloud can deduplicate.
 */
void json_logger_flush_pending(void);

/**
 * @brief Publish health diagnostics JSON
 *
 * Topic: Limelight/factory/{client_id}/health (QoS 1)
 *
 * @param data Pointer to health data structure
 */
void json_logger_health(const json_health_t *data);

/**
 * @brief Check if periodic health message should be sent (every 30s)
 *
 * @param data Pointer to current health data
 * @return true if health message was sent, false otherwise
 */
bool json_logger_health_periodic_check(const json_health_t *data);

#ifdef __cplusplus
}
#endif

#endif /* JSON_DATA_LOGGER_H */
