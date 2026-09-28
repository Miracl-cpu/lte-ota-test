/**
 * @file nvs_config.h
 * @brief Non-Volatile Storage configuration management
 *
 * Handles persistent storage of device configuration including:
 * - Device identity (ID, location)
 * - WiFi credentials
 * - MQTT settings
 * - Machine calibration parameters
 */

#ifndef NVS_CONFIG_H
#define NVS_CONFIG_H

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Configuration version (increment when structure changes)
#define NVS_CONFIG_VERSION 19

// Field length limits
#define NVS_DEVICE_ID_LEN 16
#define NVS_LOCATION_LEN 32
#define NVS_WIFI_SSID_LEN 32
#define NVS_WIFI_PASS_LEN 64
#define NVS_MQTT_HOST_LEN 64
#define NVS_MQTT_USER_LEN 32
#define NVS_MQTT_PASS_LEN 64

// Default MQTT settings (can be overridden via web UI)
#define DEFAULT_MQTT_BROKER "limelight.mvp5464.com"
#define DEFAULT_MQTT_PORT 1883
#define DEFAULT_MQTT_USERNAME "limelightit"
#define DEFAULT_MQTT_PASSWORD "4P5t3XmBhQ6C"

// Default machine settings
// Default machine settings
#define DEFAULT_CT_GAIN 1.314f
#define DEFAULT_ACTIVE_THR 0.50f
#define DEFAULT_VOLTAGE_NOMINAL 230.0f
#define DEFAULT_POWER_FACTOR 0.85f
#define DEFAULT_HYSTERESIS 0.8f
#define DEFAULT_SUSTAIN_MS 5000
#define DEFAULT_STATE_IDLE_STABILIZATION_MS DEFAULT_SUSTAIN_MS
#define DEFAULT_MACHINE_SETUP_MS 1000
#define DEFAULT_ENC_PPR 600.0f
#define DEFAULT_ENC_MPM_FACTOR 1.0f
#define DEFAULT_ENC_DIAMETER_MM 100.0f
#define DEFAULT_ENC_SCRAP_LENGTH_M 1.0f
#define DEFAULT_AUTOMIZE_CT_PART_ENABLED false
#define DEFAULT_AUTOMIZE_CT_PART_THRESHOLD_A 5.0f
#define DEFAULT_AUTOMIZE_CT_PART_STABILIZATION_MS 500
#define DEFAULT_PROXY_COUNTING_ENABLED true
#define DEFAULT_PROXY_METER_PER_COUNT 1.0f
#define DEFAULT_PROXY_SCRAP_CYCLE_TIME_S 12.0f
#define DEFAULT_PROXY_SCRAP_ENABLED false
#define DEFAULT_ENCODER_SCRAP_ENABLED false
#define DEFAULT_CYCLE_TIME_COUNT_THRESHOLD_A 5.0f
#define DEFAULT_CYCLE_TIME_COUNT_HYSTERESIS_A 1.0f
#define DEFAULT_CYCLE_SIGNAL_REARM_MS 3000
#define MAX_STATE_IDLE_STABILIZATION_MS 86400000u /* 24 hours */
#define DEFAULT_SPEED_ENABLED false
#define DEFAULT_SPEED_THRESHOLD_A 5.0f
#define DEFAULT_SPEED_MPS 1.0f
#define DEFAULT_MODBUS_CT_ENABLED false
#define DEFAULT_MODBUS_CT_THRESHOLD_KVA 0.50f
#define DEFAULT_MODBUS_CT_HYSTERESIS 0.8f
#define DEFAULT_MODBUS_CT_IDLE_STABILIZATION_MS DEFAULT_STATE_IDLE_STABILIZATION_MS

// Default cloud config sync settings
#define DEFAULT_NOMINAL_KW 10.0f
#define DEFAULT_KW_HYST_ABS 0.10f
#define DEFAULT_EXPECTED_CADENCE 2
#define DEFAULT_COUNT_FACTOR 1.0f
#define DEFAULT_IDEAL_CYCLE_SEC 12
#define DEFAULT_VIB_WARNING_MMS 4.5f
#define DEFAULT_VIB_CRITICAL_MMS 7.1f

// Max size for reason catalog JSON blob in NVS
#define NVS_REASON_CATALOG_MAX_LEN 2048
#define NVS_DURATION_MAX_PARTS 100
#define NVS_SIGNAL_COUNT_CHANNELS 2

// Field lengths for cloud config strings
#define NVS_CONFIG_ID_LEN 48
#define NVS_CATALOG_VER_LEN 24

// Default communication settings
#define DEFAULT_MQTT_KEEPALIVE 30 // Reduced from 60 to 30 seconds for faster stale connection detection
#define DEFAULT_LIVEDATA_INT 60000
#define DEFAULT_SSID_PREFIX "CT-HMI-"

/**
 * @brief State determination source
 */
typedef enum {
  NVS_STATE_SRC_CT = 0,
  NVS_STATE_SRC_SIGNAL = 1,
  NVS_STATE_SRC_ENCODER = 2,
  NVS_STATE_SRC_MODBUS_CT = 3
} nvs_state_source_t;

/**
 * @brief Signal-based state detection logic
 */
typedef enum {
  NVS_SIGNAL_LOGIC_NO = 0,
  NVS_SIGNAL_LOGIC_NC = 1
} nvs_signal_logic_t;

/**
 * @brief Production cycle-count operation source
 */
typedef enum {
  NVS_CYCLE_COUNT_SOURCE_CT = 0,
  NVS_CYCLE_COUNT_SOURCE_SIGNAL = 1
} nvs_cycle_count_source_t;

/**
 * @brief Uplink communication mode
 */
typedef enum { NVS_UPLINK_WIFI = 0, NVS_UPLINK_LTE = 1 } nvs_uplink_mode_t;

/**
 * @brief CT display/input phase mode
 */
typedef enum {
  NVS_CT_PHASE_ONE = 0,
  NVS_CT_PHASE_THREE = 1
} nvs_ct_phase_mode_t;

/**
 * @brief Vibration sensor model
 */
typedef enum {
  NVS_VIB_MODEL_RS_WZ3 = 0,
  NVS_VIB_MODEL_WTVB01_485 = 1,
  NVS_VIB_MODEL_VIBE_Q = 2
} nvs_vib_model_t;

/**
 * @brief Modbus CT / AVF meter network wiring mode
 */
typedef enum {
  NVS_MODBUS_CT_NET_3P3W = 0,
  NVS_MODBUS_CT_NET_3P4W = 1,
  NVS_MODBUS_CT_NET_1P2W = 2
} nvs_modbus_ct_network_t;

/**
 * @brief Supported RS-485 energy meter model
 */
typedef enum {
  NVS_ENERGY_METER_AVF_133_M1 = 0,
  NVS_ENERGY_METER_AVH_14_M1 = 1
} nvs_energy_meter_model_t;

/**
 * @brief Proxy production counting mode
 */
typedef enum {
  NVS_PROXY_COUNT_MODE_PARTS = 0,
  NVS_PROXY_COUNT_MODE_METER = 1
} nvs_proxy_count_mode_t;

/**
 * @brief Scrap classification source
 */
typedef enum {
  NVS_SCRAP_SOURCE_NONE = 0,
  NVS_SCRAP_SOURCE_ENCODER = 1,
  NVS_SCRAP_SOURCE_PROXY = 2,
  NVS_SCRAP_SOURCE_DURATION = 3
} nvs_scrap_source_t;

/**
 * @brief Encoder production mode
 */
typedef enum {
  NVS_ENCODER_MODE_NORMAL = 0,
  NVS_ENCODER_MODE_SCRAP = 1
} nvs_encoder_mode_t;

/**
 * @brief Encoder direction used for positive length accumulation
 */
typedef enum {
  NVS_ENCODER_DIR_CLOCKWISE = 0,
  NVS_ENCODER_DIR_ANTICLOCKWISE = 1
} nvs_encoder_direction_t;

/**
 * @brief Complete device configuration structure
 */
typedef struct {
  // ===== Device Identity =====
  char device_id[NVS_DEVICE_ID_LEN]; // e.g., "DM-004"
  char location[NVS_LOCATION_LEN];   // e.g., "Factory-A Line-3"

  // ===== WiFi Configuration =====
  char wifi_ssid[NVS_WIFI_SSID_LEN];
  char wifi_password[NVS_WIFI_PASS_LEN];
  bool wifi_configured; // true if WiFi credentials set

  // ===== MQTT Configuration =====
  char mqtt_broker[NVS_MQTT_HOST_LEN];
  uint16_t mqtt_port;
  char mqtt_username[NVS_MQTT_USER_LEN];
  char mqtt_password[NVS_MQTT_PASS_LEN];

  // ===== Machine Settings =====
  float ct_gain[3];           // CT calibration factors
  float active_threshold;     // Current threshold for ACT state
  float voltage_nominal;      // Nominal voltage
  float power_factor;         // Default power factor
  bool ignore_open_ct;        // Ignore open CTs in average
  float hysteresis_factor;    // Multiplier for exit threshold (e.g. 0.8)
  uint32_t active_sustain_ms;      // Time (ms) to confirm ACT state
  uint32_t machine_setup_time_ms;  // Time (ms) for machine warm-up before ACT (post-boot)

  // ===== Advanced Input Output =====
  uint8_t ct_enabled_mask;    // Bitmask: 0x01=PhaseA, 0x02=PhaseB, 0x04=PhaseC
  nvs_ct_phase_mode_t ct_phase_mode; // One-phase auto-detect or three-phase display
  nvs_ct_phase_mode_t state_ct_phase_mode;  // One-phase A or three-phase ACT/IDLE detection
  nvs_ct_phase_mode_t energy_ct_phase_mode; // One-phase A or three-phase energy monitoring
  uint32_t state_idle_stabilization_ms;     // Time below state threshold before IDLE
  uint8_t proxy_enabled_mask;   // Exclusive selection: 0x01=Port1, 0x02=Port2
  uint32_t proxy_debounce_ms;   // GPIO debounce for proxy sensor inputs (default 50ms)
  bool encoder_enabled;         // Enable encoder input
  float encoder_ppr;          // Pulses per revolution
  float encoder_mpm_factor;   // MPm calibration factor
  nvs_encoder_mode_t encoder_mode; // Normal encoder page or scrap count mode
  nvs_encoder_direction_t encoder_direction; // Direction that adds length
  float encoder_diameter_mm;   // Roller/pipe diameter for length calculation [mm]
  float encoder_scrap_length_m; // Target pipe length for scrap classification [m]
  bool encoder_scrap_enabled;   // Explicit encoder scrap classification enable
  nvs_state_source_t state_source; // Source for machine state logic
  nvs_signal_logic_t signal_logic; // NO/NC interpretation for signal-based state

  // ===== Proxy Production Count =====
  bool proxy_counting_enabled;       // Enable proxy production counting/MQTT/page
  nvs_proxy_count_mode_t proxy_count_mode; // Parts/count or meter/count
  float proxy_meter_per_count;       // Length added per good product [m]
  float proxy_scrap_cycle_time_s;    // HIGH duration below this is scrap [s]
  bool proxy_scrap_enabled;          // Explicit proxy scrap classification enable
  float cycle_time_count_threshold_a; // Channel A operation count threshold [A]
  float cycle_time_count_hysteresis_a; // Legacy, unused by cycle count
  nvs_cycle_count_source_t cycle_count_source; // CT threshold or signal-based operation count
  nvs_signal_logic_t cycle_signal_logic; // NO/NC interpretation for signal count
  nvs_signal_logic_t cycle_signal_logic_a; // NO/NC interpretation for Signal A count
  nvs_signal_logic_t cycle_signal_logic_b; // NO/NC interpretation for Signal B count
  uint32_t cycle_signal_rearm_ms; // Continuous inactive time before signal count rearms
  bool speed_enabled;                // Enable current-threshold speed length page
  float speed_threshold_a;           // Channel A threshold that starts length timer [A]
  float speed_mps;                   // Configured line speed [m/s]
  nvs_scrap_source_t scrap_source;   // Selected scrap calculation source

  // ===== RS-485 Energy Meter Source =====
  bool modbus_ct_enabled;             // Enable selected meter over RS485 Modbus
  nvs_energy_meter_model_t modbus_ct_model; // AVF-133-M1 (default) or AVH-14-M1
  nvs_modbus_ct_network_t modbus_ct_network; // 3P-3W, 3P-4W, or 1P-2W
  float modbus_ct_threshold_kva;       // ACT threshold in kVA
  float modbus_ct_hysteresis_factor;   // IDLE exit threshold multiplier
  uint32_t modbus_ct_idle_stabilization_ms; // Time below threshold before IDLE

  // ===== Automize CT Part Count =====
  bool automize_ct_part_enabled;       // Enable CT-threshold part counting
  float automize_ct_part_threshold_a;  // Current threshold [A]
  uint32_t automize_ct_part_stabilization_ms; // Above-threshold dwell [ms]

  // ===== Communication =====
  nvs_uplink_mode_t uplink_mode; // 0=WiFi, 1=LTE
  uint16_t mqtt_keepalive;       // Keepalive interval (sec)
  uint32_t livedata_interval_ms; // Telemetry interval (ms)
  char softap_ssid_prefix[18];   // SSID prefix for setup mode

  // ===== Cloud Config Sync =====
  char config_id[NVS_CONFIG_ID_LEN];     // e.g., "c_2026-02-10T12:48:31.000Z"
  char catalog_version[NVS_CATALOG_VER_LEN]; // e.g., "R_2026_02_10"
  float nominal_kw;                       // Nominal power (kW)
  float kw_hysteresis_abs;                // Absolute hysteresis value from cloud
  uint32_t expected_cadence_s;            // Expected cycle cadence (seconds)
  char sec_unit[12];                      // "unit", "pcs"
  float count_factor;                     // Count multiplication factor
  uint32_t ideal_cycle_time_sec;          // Ideal cycle time (seconds)
  char ct_polarity[8];                    // "normal" or "reversed"
  bool pf_clamp;                          // Power factor clamping enabled
  char smoothing_window[8];               // "off", "low", "medium", "high"

  // ===== Vibration Config (stored only, no hardware yet) =====
  bool vib_enabled;
  nvs_vib_model_t vib_model;              // RS-WZ3, WTVB01-485, or Vibe Q
  float vib_warning_mm_s;
  float vib_critical_mm_s;
  char vib_sampling[12];                  // "normal", "low", "fast"

  // ===== System Flags =====
  bool provisioned;        // true if device has been configured at least once
  uint32_t config_version; // For migration support
  uint32_t boot_count;     // Diagnostic counter

} nvs_device_config_t;

typedef struct __attribute__((packed)) {
  uint8_t valid;
  uint32_t part_number;
  uint16_t cycle_time_s;
  uint16_t operations_per_part;
} nvs_duration_part_t;

typedef struct __attribute__((packed)) {
  uint32_t magic;
  uint16_t version;
  uint8_t count_enabled;
  uint8_t selected_valid;
  uint32_t selected_part_number;
  uint32_t finished_count;
  uint16_t cycle_time_s;
  uint16_t operations_per_part;
  uint8_t signal_enabled_mask;
  uint8_t signal_selected_valid[NVS_SIGNAL_COUNT_CHANNELS];
  uint8_t reserved0;
  uint32_t signal_part_number[NVS_SIGNAL_COUNT_CHANNELS];
  uint32_t signal_finished_count[NVS_SIGNAL_COUNT_CHANNELS];
  uint16_t signal_cycle_time_s[NVS_SIGNAL_COUNT_CHANNELS];
  uint16_t signal_operations_per_part[NVS_SIGNAL_COUNT_CHANNELS];
  uint8_t reserved[8];
} nvs_duration_store_t;

/**
 * @brief Initialize NVS configuration system
 * Must be called before any other nvs_config functions
 * @return ESP_OK on success
 */
esp_err_t nvs_config_init(void);

/**
 * @brief Load configuration from NVS
 * If no config exists, loads defaults
 * @param config Pointer to config structure to fill
 * @return ESP_OK on success, ESP_ERR_NOT_FOUND if no config saved
 */
esp_err_t nvs_config_load(nvs_device_config_t *config);

/**
 * @brief Save configuration to NVS
 * @param config Pointer to config structure to save
 * @return ESP_OK on success
 */
esp_err_t nvs_config_save(const nvs_device_config_t *config);

/**
 * @brief Reset configuration to factory defaults
 * Erases all saved config and sets provisioned=false
 * @return ESP_OK on success
 */
esp_err_t nvs_config_factory_reset(void);

/**
 * @brief Load default configuration values
 * Does not write to NVS, just fills structure with defaults
 * @param config Pointer to config structure to fill
 */
void nvs_config_load_defaults(nvs_device_config_t *config);

/**
 * @brief Check if device has been provisioned
 * @return true if provisioned, false otherwise
 */
bool nvs_config_is_provisioned(void);

/**
 * @brief Check if WiFi is configured
 * @return true if WiFi credentials are set
 */
bool nvs_config_is_wifi_configured(void);

/**
 * @brief Get pointer to current loaded config (read-only)
 * @return Pointer to internal config structure, or NULL if not loaded
 * @note Unsafe for concurrent use — use nvs_config_snapshot() from tasks
 */
const nvs_device_config_t *nvs_config_get(void);

/**
 * @brief Copy current config into caller-provided buffer under mutex
 * Safe to call from any task concurrently with nvs_config_save()
 * @param out Buffer to receive the config copy
 * @return true on success, false if not loaded or out is NULL
 */
bool nvs_config_snapshot(nvs_device_config_t *out);

/**
 * @brief Update a single string field in the config
 * @param key The NVS key name
 * @param value The value to set
 * @return ESP_OK on success
 */
esp_err_t nvs_config_set_string(const char *key, const char *value);

/**
 * @brief Get the MAC address as a string
 * @param mac_out Buffer to receive MAC string (min 18 chars:
 * "AA:BB:CC:DD:EE:FF")
 * @param len Buffer length
 */
void nvs_config_get_mac_string(char *mac_out, size_t len);

/**
 * @brief Write reason catalog JSON to NVS
 * @param json_str JSON string of reasons array (e.g., "[{\"code\":\"1\",...}]")
 * @return ESP_OK on success
 */
esp_err_t nvs_config_write_reason_catalog(const char *json_str);

/**
 * @brief Read reason catalog JSON from NVS
 * @param buf Output buffer for JSON string
 * @param buf_len Buffer length (max NVS_REASON_CATALOG_MAX_LEN)
 * @return ESP_OK on success, ESP_ERR_NOT_FOUND if no catalog stored
 */
esp_err_t nvs_config_read_reason_catalog(char *buf, size_t buf_len);

/**
 * @brief Save stoppage reason to NVS (persists across reboot)
 * @param code Reason code (0-99)
 * @param l1 Category text (max 16 chars, can be empty string)
 * @param l2 Detail text (max 16 chars, can be empty string)
 * @return ESP_OK on success
 */
esp_err_t nvs_config_save_stoppage(uint8_t code, const char *l1, const char *l2);

/**
 * @brief Load saved stoppage reason from NVS
 * @param code Output reason code
 * @param l1 Output category text buffer
 * @param l1_len Size of l1 buffer
 * @param l2 Output detail text buffer
 * @param l2_len Size of l2 buffer
 * @return ESP_OK if stoppage exists, ESP_ERR_NOT_FOUND if none saved
 */
esp_err_t nvs_config_load_stoppage(uint8_t *code, char *l1, size_t l1_len,
                                    char *l2, size_t l2_len);

/**
 * @brief Clear saved stoppage reason from NVS
 * Called on state transition to ACT or new IDL
 * @return ESP_OK on success
 */
esp_err_t nvs_config_clear_stoppage(void);

/**
 * @brief Load persisted energy counters
 * @param today_kwh Output daily kWh
 * @param total_kwh Output cumulative kWh
 * @param day_key Output local day key used for midnight reset tracking
 * @return ESP_OK on success, ESP_ERR_NOT_FOUND if no counters were stored
 */
esp_err_t nvs_config_load_energy(float *today_kwh, float *total_kwh,
                                 int32_t *day_key);

/**
 * @brief Persist energy counters
 * @param today_kwh Daily kWh
 * @param total_kwh Cumulative kWh
 * @param day_key Local day key used for midnight reset tracking
 * @return ESP_OK on success
 */
esp_err_t nvs_config_save_energy(float today_kwh, float total_kwh,
                                 int32_t day_key);

/**
 * @brief Load persisted speed length total
 * @param total_length_m Output cumulative length [m]
 * @return ESP_OK on success, ESP_ERR_NOT_FOUND if no total was stored
 */
esp_err_t nvs_config_load_speed_total(float *total_length_m);

/**
 * @brief Persist speed length total
 * @param total_length_m Cumulative length [m]
 * @return ESP_OK on success
 */
esp_err_t nvs_config_save_speed_total(float total_length_m);

/* Persistent cross-mode product count; internal and adds no MQTT field. */
esp_err_t nvs_config_load_common_product_count(uint64_t *product_count);
esp_err_t nvs_config_save_common_product_count(uint64_t product_count);

/**
 * @brief Persist the latest energy reset event for field diagnostics
 */
esp_err_t nvs_config_save_energy_reset_event(const char *reason,
                                             float pre_reset_value,
                                             const char *timestamp);

void nvs_duration_load_defaults(nvs_duration_store_t *store);
esp_err_t nvs_config_load_duration(nvs_duration_store_t *store);
esp_err_t nvs_config_save_duration(const nvs_duration_store_t *store);

#ifdef __cplusplus
}
#endif

#endif // NVS_CONFIG_H
