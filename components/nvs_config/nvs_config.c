/**
 * @file nvs_config.c
 * @brief Non-Volatile Storage configuration management implementation
 */

#include "nvs_config.h"
#include "log_config.h"
#undef LOG_LOCAL_LEVEL
#define LOG_LOCAL_LEVEL LOG_LEVEL_NVS
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "nvs.h"
#include "nvs_flash.h"
#include <math.h>
#include <string.h>

static const char *TAG = "nvs_config";

// NVS namespace
#define NVS_NAMESPACE "device_cfg"

// NVS keys
#define NVS_KEY_DEVICE_ID "dev_id"
#define NVS_KEY_LOCATION "location"
#define NVS_KEY_WIFI_SSID "wifi_ssid"
#define NVS_KEY_WIFI_PASS "wifi_pass"
#define NVS_KEY_WIFI_CONFIGURED "wifi_cfg"
#define NVS_KEY_MQTT_BROKER "mqtt_host"
#define NVS_KEY_MQTT_PORT "mqtt_port"
#define NVS_KEY_MQTT_USER "mqtt_user"
#define NVS_KEY_MQTT_PASS "mqtt_pass"
#define NVS_KEY_CT_GAIN_0 "ct_gain_0"
#define NVS_KEY_CT_GAIN_1 "ct_gain_1"
#define NVS_KEY_CT_GAIN_2 "ct_gain_2"
#define NVS_KEY_ACTIVE_THR "active_thr"
#define NVS_KEY_PROVISIONED "provisioned"
#define NVS_KEY_CONFIG_VER "cfg_ver"
#define NVS_KEY_BOOT_COUNT "boot_count"

// New Keys for Extended Config
#define NVS_KEY_VOLTAGE_NOM "v_nom"
#define NVS_KEY_POWER_FACTOR "pf"
#define NVS_KEY_IGNORE_OPEN_CT "ign_open_ct"
#define NVS_KEY_HYSTERESIS "hyst_fact"
#define NVS_KEY_SUSTAIN_MS "act_sustain"
#define NVS_KEY_MACHINE_SETUP "mach_setup"
#define NVS_KEY_CT_MASK "ct_mask"
#define NVS_KEY_CT_PHASE_MODE "ct_mode"
#define NVS_KEY_STATE_CT_MODE "state_ct_mode"
#define NVS_KEY_ENERGY_CT_MODE "energy_ct_mode"
#define NVS_KEY_IDLE_STAB_MS "idle_stab_ms"
#define NVS_KEY_PROXY_MASK    "proxy_mask"
#define NVS_KEY_PROXY_DEB_MS  "proxy_deb_ms"
#define NVS_KEY_ENC_ENABLED "enc_en"
#define NVS_KEY_ENC_PPR "enc_ppr"
#define NVS_KEY_ENC_MPM_FAC "enc_mpm"
#define NVS_KEY_ENC_MODE "enc_mode"
#define NVS_KEY_ENC_DIR "enc_dir"
#define NVS_KEY_ENC_DIAMETER "enc_diam"
#define NVS_KEY_ENC_SCRAP_LEN "enc_scr_len"
#define NVS_KEY_STATE_SRC "st_src"
#define NVS_KEY_SIGNAL_LOGIC "sig_logic"
#define NVS_KEY_PROXY_COUNT_EN "pr_cnt_en"
#define NVS_KEY_PROXY_COUNT_MODE "pr_cnt_md"
#define NVS_KEY_PROXY_METER_COUNT "pr_mtr_ct"
#define NVS_KEY_PROXY_SCRAP_CYCLE "pr_scr_cy"
#define NVS_KEY_CYCLE_CNT_THR_A "cy_thr_a"
#define NVS_KEY_CYCLE_CNT_HYST_A "cy_hyst_a"
#define NVS_KEY_CYCLE_CNT_SRC "cy_src"
#define NVS_KEY_CYCLE_SIG_LOGIC "cy_sig_lg"
#define NVS_KEY_CYCLE_SIG_LOGIC_A "cy_sig_a"
#define NVS_KEY_CYCLE_SIG_LOGIC_B "cy_sig_b"
#define NVS_KEY_CYCLE_SIG_REARM "cy_sig_re"
#define NVS_KEY_SPEED_ENABLED "spd_en"
#define NVS_KEY_SPEED_THR_A "spd_thr_a"
#define NVS_KEY_SPEED_MPS "spd_mps"
#define NVS_KEY_SCRAP_SOURCE "scrap_src"
#define NVS_KEY_PROXY_SCRAP_EN "pr_scr_en"
#define NVS_KEY_ENC_SCRAP_EN "enc_scr_en"
#define NVS_KEY_MODBUS_CT_EN "mbct_en"
#define NVS_KEY_MODBUS_CT_MODEL "mbct_model"
#define NVS_KEY_MODBUS_CT_NET "mbct_net"
#define NVS_KEY_MODBUS_CT_THR "mbct_thr"
#define NVS_KEY_MODBUS_CT_HYST "mbct_hyst"
#define NVS_KEY_MODBUS_CT_IDLE "mbct_idle"
#define NVS_KEY_AUTO_CT_PART_EN "auto_ct_en"
#define NVS_KEY_AUTO_CT_PART_THR "auto_ct_thr"
#define NVS_KEY_AUTO_CT_PART_STAB "auto_ct_st"
#define NVS_KEY_UPLINK_MODE "uplink"
#define NVS_KEY_MQTT_KEEPALIVE "mqtt_ka"
#define NVS_KEY_LIVEDATA_INT "live_int"
#define NVS_KEY_SSID_PREFIX "ap_prefix"

// Cloud Config Sync keys
#define NVS_KEY_CFG_ID "cfg_id"
#define NVS_KEY_CATALOG_VER "cat_ver"
#define NVS_KEY_NOMINAL_KW "nom_kw"
#define NVS_KEY_KW_HYST_ABS "kw_hyst"
#define NVS_KEY_EXP_CADENCE "exp_cad"
#define NVS_KEY_SEC_UNIT "sec_unit"
#define NVS_KEY_COUNT_FACTOR "cnt_fact"
#define NVS_KEY_IDEAL_CYCLE "ideal_cyc"
#define NVS_KEY_CT_POLARITY "ct_pol"
#define NVS_KEY_PF_CLAMP "pf_clamp"
#define NVS_KEY_SMOOTH_WIN "smooth"
#define NVS_KEY_VIB_EN "vib_en"
#define NVS_KEY_VIB_MODEL "vib_model"
#define NVS_KEY_VIB_WARN "vib_warn"
#define NVS_KEY_VIB_CRIT "vib_crit"
#define NVS_KEY_VIB_SAMP "vib_samp"
#define NVS_KEY_REASON_CAT "reason_cat"

// Stoppage reason persistence keys (survives reboot)
#define NVS_KEY_STP_CODE "stp_code"
#define NVS_KEY_STP_L1 "stp_l1"
#define NVS_KEY_STP_L2 "stp_l2"

// Energy counter persistence keys
#define NVS_KEY_ENERGY_TODAY "en_today"
#define NVS_KEY_ENERGY_TOTAL "en_total"
#define NVS_KEY_ENERGY_DAY "en_day"
#define NVS_KEY_ENERGY_RST_REASON "en_rs_rsn"
#define NVS_KEY_ENERGY_RST_VALUE "en_rs_val"
#define NVS_KEY_ENERGY_RST_TS "en_rs_ts"
#define NVS_KEY_DURATION_STORE "dur_store"
#define NVS_KEY_SPEED_TOTAL "spd_total"
#define NVS_KEY_COMMON_PROD_COUNT "prod_count"

#define NVS_DURATION_MAGIC 0x44555231u
#define NVS_DURATION_VERSION 3

typedef struct __attribute__((packed)) {
  uint32_t magic;
  uint16_t version;
  uint8_t count_enabled;
  uint8_t selected_valid;
  uint32_t selected_part_number;
  uint32_t finished_count;
  uint16_t cycle_time_s;
  uint16_t operations_per_part;
  uint8_t reserved[8];
} nvs_duration_store_v2_t;

// Internal state
static nvs_device_config_t s_config;
static bool s_initialized = false;
static bool s_config_loaded = false;
static SemaphoreHandle_t s_mutex = NULL;
// Published RAM snapshot: readers never wait for a flash erase/commit.
static nvs_device_config_t s_read_snapshot;
static bool s_read_snapshot_valid;
static portMUX_TYPE s_snapshot_mux = portMUX_INITIALIZER_UNLOCKED;
static void publish_config_snapshot(void) {
  portENTER_CRITICAL(&s_snapshot_mux);
  s_read_snapshot = s_config;
  s_read_snapshot_valid = true;
  portEXIT_CRITICAL(&s_snapshot_mux);
}

// Helper macros for mutex
#define LOCK()                                                                 \
  do {                                                                         \
    if (s_mutex) {                                                             \
      if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(5000)) != pdTRUE) {          \
        ESP_LOGE(TAG, "NVS mutex timeout — restarting");                      \
        esp_restart();                                                         \
      }                                                                        \
    }                                                                          \
  } while (0)
#define UNLOCK()                                                               \
  do {                                                                         \
    if (s_mutex)                                                               \
      xSemaphoreGive(s_mutex);                                                 \
  } while (0)

/**
 * @brief Read a string from NVS with fallback to default
 */
static esp_err_t nvs_read_string(nvs_handle_t handle, const char *key,
                                 char *out, size_t max_len, const char *def) {
  size_t len = max_len;
  esp_err_t ret = nvs_get_str(handle, key, out, &len);
  if (ret == ESP_ERR_NVS_NOT_FOUND) {
    if (def) {
      strncpy(out, def, max_len - 1);
      out[max_len - 1] = '\0';
    } else {
      out[0] = '\0';
    }
    return ESP_ERR_NVS_NOT_FOUND;
  } else if (ret != ESP_OK) {
    ESP_LOGW(TAG, "Failed to read %s: %s", key, esp_err_to_name(ret));
    if (def) {
      strncpy(out, def, max_len - 1);
      out[max_len - 1] = '\0';
    }
  }
  return ret;
}

/**
 * @brief Write a string to NVS
 */
static esp_err_t nvs_write_string(nvs_handle_t handle, const char *key,
                                  const char *value) {
  esp_err_t ret = nvs_set_str(handle, key, value);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to write %s: %s", key, esp_err_to_name(ret));
  }
  return ret;
}

/**
 * @brief Read a uint8_t from NVS with default
 */
static uint8_t nvs_read_u8(nvs_handle_t handle, const char *key, uint8_t def) {
  uint8_t value = def;
  esp_err_t ret = nvs_get_u8(handle, key, &value);
  if (ret != ESP_OK && ret != ESP_ERR_NVS_NOT_FOUND) {
    ESP_LOGW(TAG, "Failed to read %s: %s", key, esp_err_to_name(ret));
  }
  return value;
}

/**
 * @brief Read a uint16_t from NVS with default
 */
static uint16_t nvs_read_u16(nvs_handle_t handle, const char *key,
                             uint16_t def) {
  uint16_t value = def;
  esp_err_t ret = nvs_get_u16(handle, key, &value);
  if (ret != ESP_OK && ret != ESP_ERR_NVS_NOT_FOUND) {
    ESP_LOGW(TAG, "Failed to read %s: %s", key, esp_err_to_name(ret));
  }
  return value;
}

/**
 * @brief Read a uint32_t from NVS with default
 */
static uint32_t nvs_read_u32(nvs_handle_t handle, const char *key,
                             uint32_t def) {
  uint32_t value = def;
  esp_err_t ret = nvs_get_u32(handle, key, &value);
  if (ret != ESP_OK && ret != ESP_ERR_NVS_NOT_FOUND) {
    ESP_LOGW(TAG, "Failed to read %s: %s", key, esp_err_to_name(ret));
  }
  return value;
}

/**
 * @brief Read a float from NVS (stored as blob) with default
 */
static float nvs_read_float(nvs_handle_t handle, const char *key, float def) {
  float value = def;
  size_t len = sizeof(float);
  esp_err_t ret = nvs_get_blob(handle, key, &value, &len);
  if (ret != ESP_OK && ret != ESP_ERR_NVS_NOT_FOUND) {
    ESP_LOGW(TAG, "Failed to read %s: %s", key, esp_err_to_name(ret));
  }
  return value;
}

/**
 * @brief Write a float to NVS (as blob)
 */
static esp_err_t nvs_write_float(nvs_handle_t handle, const char *key,
                                 float value) {
  esp_err_t ret = nvs_set_blob(handle, key, &value, sizeof(float));
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to write %s: %s", key, esp_err_to_name(ret));
  }
  return ret;
}

esp_err_t nvs_config_init(void) {
  if (s_initialized) {
    return ESP_OK;
  }

  // Initialize NVS flash
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_LOGW(TAG, "NVS partition truncated, erasing...");
    ret = nvs_flash_erase();
    if (ret != ESP_OK) {
      ESP_LOGE(TAG, "Failed to erase NVS flash: %s", esp_err_to_name(ret));
      return ret;
    }
    ret = nvs_flash_init();
  }
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to init NVS flash: %s", esp_err_to_name(ret));
    return ret;
  }

  // Create mutex
  s_mutex = xSemaphoreCreateMutex();
  if (s_mutex == NULL) {
    ESP_LOGE(TAG, "Failed to create mutex");
    return ESP_ERR_NO_MEM;
  }

  // Load defaults initially
  nvs_config_load_defaults(&s_config);

  s_initialized = true;
  ESP_LOGI(TAG, "NVS config initialized");
  return ESP_OK;
}

void nvs_config_load_defaults(nvs_device_config_t *config) {
  if (config == NULL)
    return;

  memset(config, 0, sizeof(nvs_device_config_t));

  // Device identity - derive default from last 3 bytes of base MAC
  {
    uint8_t mac[6];
    esp_base_mac_addr_get(mac);
    snprintf(config->device_id, NVS_DEVICE_ID_LEN, "DM-%02X%02X%02X",
             mac[3], mac[4], mac[5]);
  }
  config->location[0] = '\0';

  // WiFi - not configured by default
  config->wifi_ssid[0] = '\0';
  config->wifi_password[0] = '\0';
  config->wifi_configured = false;

  // MQTT - use defaults
  strncpy(config->mqtt_broker, DEFAULT_MQTT_BROKER, NVS_MQTT_HOST_LEN - 1);
  config->mqtt_port = DEFAULT_MQTT_PORT;
  strncpy(config->mqtt_username, DEFAULT_MQTT_USERNAME, NVS_MQTT_USER_LEN - 1);
  strncpy(config->mqtt_password, DEFAULT_MQTT_PASSWORD, NVS_MQTT_PASS_LEN - 1);

  // Machine settings - defaults
  config->ct_gain[0] = DEFAULT_CT_GAIN;
  config->ct_gain[1] = DEFAULT_CT_GAIN;
  config->ct_gain[2] = DEFAULT_CT_GAIN;
  config->active_threshold = DEFAULT_ACTIVE_THR;
  config->voltage_nominal = DEFAULT_VOLTAGE_NOMINAL;
  config->power_factor = DEFAULT_POWER_FACTOR;
  config->ignore_open_ct = true;
  config->hysteresis_factor = DEFAULT_HYSTERESIS;
  config->active_sustain_ms = DEFAULT_SUSTAIN_MS;
  config->machine_setup_time_ms = DEFAULT_MACHINE_SETUP_MS;

  // Advanced Input defaults
  config->ct_enabled_mask = 0x07; // All 3 phases enabled
  config->ct_phase_mode = NVS_CT_PHASE_THREE;
  config->state_ct_phase_mode = NVS_CT_PHASE_THREE;
  config->energy_ct_phase_mode = NVS_CT_PHASE_THREE;
  config->state_idle_stabilization_ms = DEFAULT_STATE_IDLE_STABILIZATION_MS;
  config->proxy_enabled_mask = 0x01; // Proxy A selected by default
  config->proxy_debounce_ms  = 50;
  config->encoder_enabled = true;
  config->encoder_ppr = DEFAULT_ENC_PPR;
  config->encoder_mpm_factor = DEFAULT_ENC_MPM_FACTOR;
  config->encoder_mode = NVS_ENCODER_MODE_NORMAL;
  config->encoder_direction = NVS_ENCODER_DIR_CLOCKWISE;
  config->encoder_diameter_mm = DEFAULT_ENC_DIAMETER_MM;
  config->encoder_scrap_length_m = DEFAULT_ENC_SCRAP_LENGTH_M;
  config->encoder_scrap_enabled = DEFAULT_ENCODER_SCRAP_ENABLED;
  config->state_source = NVS_STATE_SRC_CT;
  config->signal_logic = NVS_SIGNAL_LOGIC_NO;
  config->proxy_counting_enabled = DEFAULT_PROXY_COUNTING_ENABLED;
  config->proxy_count_mode = NVS_PROXY_COUNT_MODE_PARTS;
  config->proxy_meter_per_count = DEFAULT_PROXY_METER_PER_COUNT;
  config->proxy_scrap_cycle_time_s = DEFAULT_PROXY_SCRAP_CYCLE_TIME_S;
  config->proxy_scrap_enabled = DEFAULT_PROXY_SCRAP_ENABLED;
  config->cycle_time_count_threshold_a =
      DEFAULT_CYCLE_TIME_COUNT_THRESHOLD_A;
  config->cycle_time_count_hysteresis_a =
      DEFAULT_CYCLE_TIME_COUNT_HYSTERESIS_A;
  config->cycle_count_source = NVS_CYCLE_COUNT_SOURCE_CT;
  config->cycle_signal_logic = NVS_SIGNAL_LOGIC_NO;
  config->cycle_signal_logic_a = NVS_SIGNAL_LOGIC_NO;
  config->cycle_signal_logic_b = NVS_SIGNAL_LOGIC_NO;
  config->cycle_signal_rearm_ms = DEFAULT_CYCLE_SIGNAL_REARM_MS;
  config->speed_enabled = DEFAULT_SPEED_ENABLED;
  config->speed_threshold_a = DEFAULT_SPEED_THRESHOLD_A;
  config->speed_mps = DEFAULT_SPEED_MPS;
  config->scrap_source = NVS_SCRAP_SOURCE_PROXY;
  config->modbus_ct_enabled = DEFAULT_MODBUS_CT_ENABLED;
  config->modbus_ct_model = NVS_ENERGY_METER_AVF_133_M1;
  config->modbus_ct_network = NVS_MODBUS_CT_NET_3P4W;
  config->modbus_ct_threshold_kva = DEFAULT_MODBUS_CT_THRESHOLD_KVA;
  config->modbus_ct_hysteresis_factor = DEFAULT_MODBUS_CT_HYSTERESIS;
  config->modbus_ct_idle_stabilization_ms =
      DEFAULT_MODBUS_CT_IDLE_STABILIZATION_MS;
  config->automize_ct_part_enabled = DEFAULT_AUTOMIZE_CT_PART_ENABLED;
  config->automize_ct_part_threshold_a = DEFAULT_AUTOMIZE_CT_PART_THRESHOLD_A;
  config->automize_ct_part_stabilization_ms =
      DEFAULT_AUTOMIZE_CT_PART_STABILIZATION_MS;

  // Communication defaults
  config->uplink_mode = NVS_UPLINK_WIFI;  // Default: WiFi
  config->mqtt_keepalive = DEFAULT_MQTT_KEEPALIVE;
  config->livedata_interval_ms = DEFAULT_LIVEDATA_INT;
  strncpy(config->softap_ssid_prefix, DEFAULT_SSID_PREFIX,
          sizeof(config->softap_ssid_prefix) - 1);

  // Cloud config sync defaults
  config->config_id[0] = '\0';
  config->catalog_version[0] = '\0';
  config->nominal_kw = DEFAULT_NOMINAL_KW;
  config->kw_hysteresis_abs = DEFAULT_KW_HYST_ABS;
  config->expected_cadence_s = DEFAULT_EXPECTED_CADENCE;
  strncpy(config->sec_unit, "unit", sizeof(config->sec_unit) - 1);
  config->count_factor = DEFAULT_COUNT_FACTOR;
  config->ideal_cycle_time_sec = DEFAULT_IDEAL_CYCLE_SEC;
  strncpy(config->ct_polarity, "normal", sizeof(config->ct_polarity) - 1);
  config->pf_clamp = true;
  strncpy(config->smoothing_window, "off", sizeof(config->smoothing_window) - 1);

  // Vibration defaults (stored only)
  config->vib_enabled = true;
  config->vib_model = NVS_VIB_MODEL_VIBE_Q;
  config->vib_warning_mm_s = DEFAULT_VIB_WARNING_MMS;
  config->vib_critical_mm_s = DEFAULT_VIB_CRITICAL_MMS;
  strncpy(config->vib_sampling, "normal", sizeof(config->vib_sampling) - 1);

  // System flags
  config->provisioned = false;
  config->config_version = NVS_CONFIG_VERSION;
  config->boot_count = 0;
}

esp_err_t nvs_config_load(nvs_device_config_t *config) {
  if (!s_initialized) {
    ESP_LOGE(TAG, "NVS config not initialized");
    return ESP_ERR_INVALID_STATE;
  }
  if (config == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  LOCK();

  // Start with defaults
  nvs_config_load_defaults(config);

  // Open NVS handle
  nvs_handle_t handle;
  esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);
  if (ret == ESP_ERR_NVS_NOT_FOUND) {
    ESP_LOGW(TAG, "No saved config found, using defaults");
    memcpy(&s_config, config, sizeof(nvs_device_config_t));
    s_config_loaded = true;
  publish_config_snapshot();
    UNLOCK();
    return ESP_ERR_NOT_FOUND;
  } else if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to open NVS: %s", esp_err_to_name(ret));
    UNLOCK();
    return ret;
  }

  // Check config version for migration
  uint32_t saved_version = nvs_read_u32(handle, NVS_KEY_CONFIG_VER, 0);
  bool migration_needed = (saved_version != NVS_CONFIG_VERSION);
  if (migration_needed) {
    ESP_LOGW(TAG, "[NVS_MIGRATION] Config v%lu → v%d detected — will migrate after load",
             (unsigned long)saved_version, NVS_CONFIG_VERSION);
  }

  // Load device identity
  nvs_read_string(handle, NVS_KEY_DEVICE_ID, config->device_id,
                  NVS_DEVICE_ID_LEN, "");
  nvs_read_string(handle, NVS_KEY_LOCATION, config->location, NVS_LOCATION_LEN,
                  "");

  // Load WiFi config
  nvs_read_string(handle, NVS_KEY_WIFI_SSID, config->wifi_ssid,
                  NVS_WIFI_SSID_LEN, "");
  nvs_read_string(handle, NVS_KEY_WIFI_PASS, config->wifi_password,
                  NVS_WIFI_PASS_LEN, "");
  config->wifi_configured =
      nvs_read_u8(handle, NVS_KEY_WIFI_CONFIGURED, 0) != 0;

  // Load MQTT config
  nvs_read_string(handle, NVS_KEY_MQTT_BROKER, config->mqtt_broker,
                  NVS_MQTT_HOST_LEN, DEFAULT_MQTT_BROKER);
  config->mqtt_port =
      nvs_read_u16(handle, NVS_KEY_MQTT_PORT, DEFAULT_MQTT_PORT);
  nvs_read_string(handle, NVS_KEY_MQTT_USER, config->mqtt_username,
                  NVS_MQTT_USER_LEN, DEFAULT_MQTT_USERNAME);
  nvs_read_string(handle, NVS_KEY_MQTT_PASS, config->mqtt_password,
                  NVS_MQTT_PASS_LEN, DEFAULT_MQTT_PASSWORD);

  // Load machine settings
  config->ct_gain[0] =
      nvs_read_float(handle, NVS_KEY_CT_GAIN_0, DEFAULT_CT_GAIN);
  config->ct_gain[1] =
      nvs_read_float(handle, NVS_KEY_CT_GAIN_1, DEFAULT_CT_GAIN);
  config->ct_gain[2] =
      nvs_read_float(handle, NVS_KEY_CT_GAIN_2, DEFAULT_CT_GAIN);
  config->active_threshold =
      nvs_read_float(handle, NVS_KEY_ACTIVE_THR, DEFAULT_ACTIVE_THR);
  config->voltage_nominal =
      nvs_read_float(handle, NVS_KEY_VOLTAGE_NOM, DEFAULT_VOLTAGE_NOMINAL);
  config->power_factor =
      nvs_read_float(handle, NVS_KEY_POWER_FACTOR, DEFAULT_POWER_FACTOR);
  config->ignore_open_ct = nvs_read_u8(handle, NVS_KEY_IGNORE_OPEN_CT, 1) != 0;
  config->hysteresis_factor =
      nvs_read_float(handle, NVS_KEY_HYSTERESIS, DEFAULT_HYSTERESIS);
  config->active_sustain_ms =
      nvs_read_u32(handle, NVS_KEY_SUSTAIN_MS, DEFAULT_SUSTAIN_MS);
  config->machine_setup_time_ms =
      nvs_read_u32(handle, NVS_KEY_MACHINE_SETUP, DEFAULT_MACHINE_SETUP_MS);

  // Load advanced inputs
  config->ct_enabled_mask = nvs_read_u8(handle, NVS_KEY_CT_MASK, 0x07);
  config->ct_phase_mode = (nvs_ct_phase_mode_t)nvs_read_u8(
      handle, NVS_KEY_CT_PHASE_MODE, (uint8_t)NVS_CT_PHASE_THREE);
  if (config->ct_phase_mode != NVS_CT_PHASE_ONE &&
      config->ct_phase_mode != NVS_CT_PHASE_THREE) {
    config->ct_phase_mode = NVS_CT_PHASE_THREE;
  }
  config->state_ct_phase_mode = (nvs_ct_phase_mode_t)nvs_read_u8(
      handle, NVS_KEY_STATE_CT_MODE, (uint8_t)NVS_CT_PHASE_THREE);
  if (config->state_ct_phase_mode != NVS_CT_PHASE_ONE &&
      config->state_ct_phase_mode != NVS_CT_PHASE_THREE) {
    config->state_ct_phase_mode = NVS_CT_PHASE_THREE;
  }
  config->energy_ct_phase_mode = (nvs_ct_phase_mode_t)nvs_read_u8(
      handle, NVS_KEY_ENERGY_CT_MODE, (uint8_t)NVS_CT_PHASE_THREE);
  if (config->energy_ct_phase_mode != NVS_CT_PHASE_ONE &&
      config->energy_ct_phase_mode != NVS_CT_PHASE_THREE) {
    config->energy_ct_phase_mode = NVS_CT_PHASE_THREE;
  }
  config->state_idle_stabilization_ms = nvs_read_u32(
      handle, NVS_KEY_IDLE_STAB_MS, DEFAULT_STATE_IDLE_STABILIZATION_MS);
  if (config->state_idle_stabilization_ms < 1000 ||
      config->state_idle_stabilization_ms > MAX_STATE_IDLE_STABILIZATION_MS) {
    config->state_idle_stabilization_ms = DEFAULT_STATE_IDLE_STABILIZATION_MS;
  }
  config->proxy_enabled_mask = nvs_read_u8(handle, NVS_KEY_PROXY_MASK, 0x01);
  /* Proxy A/B are mutually exclusive production inputs.  Prefer A when
   * migrating the legacy 0x03 value; preserve an existing B-only choice. */
  config->proxy_enabled_mask =
      (config->proxy_enabled_mask & 0x01) ? 0x01
      : (config->proxy_enabled_mask & 0x02) ? 0x02 : 0x01;
  config->proxy_debounce_ms  = nvs_read_u32(handle, NVS_KEY_PROXY_DEB_MS, 50);
  config->encoder_enabled = nvs_read_u8(handle, NVS_KEY_ENC_ENABLED, 1) != 0;
  config->encoder_ppr =
      nvs_read_float(handle, NVS_KEY_ENC_PPR, DEFAULT_ENC_PPR);
  config->encoder_mpm_factor =
      nvs_read_float(handle, NVS_KEY_ENC_MPM_FAC, DEFAULT_ENC_MPM_FACTOR);
  config->encoder_mode = (nvs_encoder_mode_t)nvs_read_u8(
      handle, NVS_KEY_ENC_MODE, (uint8_t)NVS_ENCODER_MODE_NORMAL);
  if (config->encoder_mode != NVS_ENCODER_MODE_NORMAL &&
      config->encoder_mode != NVS_ENCODER_MODE_SCRAP) {
    config->encoder_mode = NVS_ENCODER_MODE_NORMAL;
  }
  config->encoder_direction = (nvs_encoder_direction_t)nvs_read_u8(
      handle, NVS_KEY_ENC_DIR, (uint8_t)NVS_ENCODER_DIR_CLOCKWISE);
  if (config->encoder_direction != NVS_ENCODER_DIR_CLOCKWISE &&
      config->encoder_direction != NVS_ENCODER_DIR_ANTICLOCKWISE) {
    config->encoder_direction = NVS_ENCODER_DIR_CLOCKWISE;
  }
  config->encoder_diameter_mm =
      nvs_read_float(handle, NVS_KEY_ENC_DIAMETER,
                     DEFAULT_ENC_DIAMETER_MM);
  config->encoder_scrap_length_m =
      nvs_read_float(handle, NVS_KEY_ENC_SCRAP_LEN,
                     DEFAULT_ENC_SCRAP_LENGTH_M);
  config->encoder_scrap_enabled =
      nvs_read_u8(handle, NVS_KEY_ENC_SCRAP_EN,
                  DEFAULT_ENCODER_SCRAP_ENABLED ? 1 : 0) != 0;
  config->state_source = (nvs_state_source_t)nvs_read_u8(
      handle, NVS_KEY_STATE_SRC, (uint8_t)NVS_STATE_SRC_CT);
  if (config->state_source != NVS_STATE_SRC_CT &&
      config->state_source != NVS_STATE_SRC_SIGNAL &&
      config->state_source != NVS_STATE_SRC_MODBUS_CT) {
    config->state_source = NVS_STATE_SRC_CT;
  }
  config->signal_logic = (nvs_signal_logic_t)nvs_read_u8(
      handle, NVS_KEY_SIGNAL_LOGIC, (uint8_t)NVS_SIGNAL_LOGIC_NO);
  if (config->signal_logic != NVS_SIGNAL_LOGIC_NO &&
      config->signal_logic != NVS_SIGNAL_LOGIC_NC) {
    config->signal_logic = NVS_SIGNAL_LOGIC_NO;
  }
  config->proxy_counting_enabled =
      nvs_read_u8(handle, NVS_KEY_PROXY_COUNT_EN,
                  DEFAULT_PROXY_COUNTING_ENABLED ? 1 : 0) != 0;
  config->proxy_count_mode = (nvs_proxy_count_mode_t)nvs_read_u8(
      handle, NVS_KEY_PROXY_COUNT_MODE,
      (uint8_t)NVS_PROXY_COUNT_MODE_PARTS);
  if (config->proxy_count_mode != NVS_PROXY_COUNT_MODE_PARTS &&
      config->proxy_count_mode != NVS_PROXY_COUNT_MODE_METER) {
    config->proxy_count_mode = NVS_PROXY_COUNT_MODE_PARTS;
  }
  config->proxy_meter_per_count =
      nvs_read_float(handle, NVS_KEY_PROXY_METER_COUNT,
                     DEFAULT_PROXY_METER_PER_COUNT);
  config->proxy_scrap_cycle_time_s =
      nvs_read_float(handle, NVS_KEY_PROXY_SCRAP_CYCLE,
                     DEFAULT_PROXY_SCRAP_CYCLE_TIME_S);
  config->proxy_scrap_enabled =
      nvs_read_u8(handle, NVS_KEY_PROXY_SCRAP_EN,
                  DEFAULT_PROXY_SCRAP_ENABLED ? 1 : 0) != 0;
  config->cycle_time_count_threshold_a =
      nvs_read_float(handle, NVS_KEY_CYCLE_CNT_THR_A,
                     DEFAULT_CYCLE_TIME_COUNT_THRESHOLD_A);
  config->cycle_time_count_hysteresis_a =
      nvs_read_float(handle, NVS_KEY_CYCLE_CNT_HYST_A,
                     DEFAULT_CYCLE_TIME_COUNT_HYSTERESIS_A);
  if (!isfinite(config->cycle_time_count_threshold_a) ||
      config->cycle_time_count_threshold_a <= 0.0f ||
      config->cycle_time_count_threshold_a > 1000.0f) {
    config->cycle_time_count_threshold_a =
        DEFAULT_CYCLE_TIME_COUNT_THRESHOLD_A;
  }
  if (!isfinite(config->cycle_time_count_hysteresis_a) ||
      config->cycle_time_count_hysteresis_a < 0.0f ||
      config->cycle_time_count_hysteresis_a >=
          config->cycle_time_count_threshold_a) {
    config->cycle_time_count_hysteresis_a =
        DEFAULT_CYCLE_TIME_COUNT_HYSTERESIS_A;
    if (config->cycle_time_count_hysteresis_a >=
        config->cycle_time_count_threshold_a) {
      config->cycle_time_count_hysteresis_a =
          config->cycle_time_count_threshold_a * 0.5f;
    }
  }
  config->cycle_count_source = (nvs_cycle_count_source_t)nvs_read_u8(
      handle, NVS_KEY_CYCLE_CNT_SRC, (uint8_t)NVS_CYCLE_COUNT_SOURCE_CT);
  if (config->cycle_count_source != NVS_CYCLE_COUNT_SOURCE_CT &&
      config->cycle_count_source != NVS_CYCLE_COUNT_SOURCE_SIGNAL) {
    config->cycle_count_source = NVS_CYCLE_COUNT_SOURCE_CT;
  }
  config->cycle_signal_logic = (nvs_signal_logic_t)nvs_read_u8(
      handle, NVS_KEY_CYCLE_SIG_LOGIC, (uint8_t)NVS_SIGNAL_LOGIC_NO);
  if (config->cycle_signal_logic != NVS_SIGNAL_LOGIC_NO &&
      config->cycle_signal_logic != NVS_SIGNAL_LOGIC_NC) {
    config->cycle_signal_logic = NVS_SIGNAL_LOGIC_NO;
  }
  config->cycle_signal_logic_a = (nvs_signal_logic_t)nvs_read_u8(
      handle, NVS_KEY_CYCLE_SIG_LOGIC_A, (uint8_t)config->cycle_signal_logic);
  if (config->cycle_signal_logic_a != NVS_SIGNAL_LOGIC_NO &&
      config->cycle_signal_logic_a != NVS_SIGNAL_LOGIC_NC) {
    config->cycle_signal_logic_a = config->cycle_signal_logic;
  }
  config->cycle_signal_logic_b = (nvs_signal_logic_t)nvs_read_u8(
      handle, NVS_KEY_CYCLE_SIG_LOGIC_B, (uint8_t)config->cycle_signal_logic);
  if (config->cycle_signal_logic_b != NVS_SIGNAL_LOGIC_NO &&
      config->cycle_signal_logic_b != NVS_SIGNAL_LOGIC_NC) {
    config->cycle_signal_logic_b = config->cycle_signal_logic;
  }
  config->cycle_signal_rearm_ms =
      nvs_read_u32(handle, NVS_KEY_CYCLE_SIG_REARM,
                   DEFAULT_CYCLE_SIGNAL_REARM_MS);
  if (config->cycle_signal_rearm_ms < 100 ||
      config->cycle_signal_rearm_ms > 60000) {
    config->cycle_signal_rearm_ms = DEFAULT_CYCLE_SIGNAL_REARM_MS;
  }
  config->speed_enabled =
      nvs_read_u8(handle, NVS_KEY_SPEED_ENABLED,
                  DEFAULT_SPEED_ENABLED ? 1 : 0) != 0;
  config->speed_threshold_a =
      nvs_read_float(handle, NVS_KEY_SPEED_THR_A,
                     DEFAULT_SPEED_THRESHOLD_A);
  if (!isfinite(config->speed_threshold_a) ||
      config->speed_threshold_a <= 0.0f ||
      config->speed_threshold_a > 1000.0f) {
    config->speed_threshold_a = DEFAULT_SPEED_THRESHOLD_A;
  }
  config->speed_mps =
      nvs_read_float(handle, NVS_KEY_SPEED_MPS, DEFAULT_SPEED_MPS);
  if (!isfinite(config->speed_mps) || config->speed_mps <= 0.0f ||
      config->speed_mps > 1000.0f) {
    config->speed_mps = DEFAULT_SPEED_MPS;
  }
  config->scrap_source = (nvs_scrap_source_t)nvs_read_u8(
      handle, NVS_KEY_SCRAP_SOURCE, (uint8_t)NVS_SCRAP_SOURCE_PROXY);
  if (config->scrap_source != NVS_SCRAP_SOURCE_NONE &&
      config->scrap_source != NVS_SCRAP_SOURCE_ENCODER &&
      config->scrap_source != NVS_SCRAP_SOURCE_PROXY) {
    config->scrap_source = NVS_SCRAP_SOURCE_PROXY;
  }
  config->modbus_ct_enabled =
      nvs_read_u8(handle, NVS_KEY_MODBUS_CT_EN,
                  DEFAULT_MODBUS_CT_ENABLED ? 1 : 0) != 0;
  /* Legacy images used CT + modbus_ct_enabled to mean MODBUS state source. */
  if (config->modbus_ct_enabled && config->state_source == NVS_STATE_SRC_CT) {
    config->state_source = NVS_STATE_SRC_MODBUS_CT;
  } else if (!config->modbus_ct_enabled &&
             config->state_source == NVS_STATE_SRC_MODBUS_CT) {
    config->state_source = NVS_STATE_SRC_CT;
  }
  config->modbus_ct_model = (nvs_energy_meter_model_t)nvs_read_u8(
      handle, NVS_KEY_MODBUS_CT_MODEL,
      (uint8_t)NVS_ENERGY_METER_AVF_133_M1);
  if (config->modbus_ct_model != NVS_ENERGY_METER_AVF_133_M1 &&
      config->modbus_ct_model != NVS_ENERGY_METER_AVH_14_M1) {
    config->modbus_ct_model = NVS_ENERGY_METER_AVF_133_M1;
  }
  config->modbus_ct_network = (nvs_modbus_ct_network_t)nvs_read_u8(
      handle, NVS_KEY_MODBUS_CT_NET, (uint8_t)NVS_MODBUS_CT_NET_3P4W);
  if (config->modbus_ct_network != NVS_MODBUS_CT_NET_3P3W &&
      config->modbus_ct_network != NVS_MODBUS_CT_NET_3P4W &&
      config->modbus_ct_network != NVS_MODBUS_CT_NET_1P2W) {
    config->modbus_ct_network = NVS_MODBUS_CT_NET_3P4W;
  }
  config->modbus_ct_threshold_kva =
      nvs_read_float(handle, NVS_KEY_MODBUS_CT_THR,
                     DEFAULT_MODBUS_CT_THRESHOLD_KVA);
  if (!isfinite(config->modbus_ct_threshold_kva) ||
      config->modbus_ct_threshold_kva <= 0.0f ||
      config->modbus_ct_threshold_kva > 1000.0f) {
    config->modbus_ct_threshold_kva = DEFAULT_MODBUS_CT_THRESHOLD_KVA;
  }
  config->modbus_ct_hysteresis_factor =
      nvs_read_float(handle, NVS_KEY_MODBUS_CT_HYST,
                     DEFAULT_MODBUS_CT_HYSTERESIS);
  if (!isfinite(config->modbus_ct_hysteresis_factor) ||
      config->modbus_ct_hysteresis_factor < 0.1f ||
      config->modbus_ct_hysteresis_factor > 1.0f) {
    config->modbus_ct_hysteresis_factor = DEFAULT_MODBUS_CT_HYSTERESIS;
  }
  config->modbus_ct_idle_stabilization_ms =
      nvs_read_u32(handle, NVS_KEY_MODBUS_CT_IDLE,
                   DEFAULT_MODBUS_CT_IDLE_STABILIZATION_MS);
  if (config->modbus_ct_idle_stabilization_ms < 1000 ||
      config->modbus_ct_idle_stabilization_ms >
          MAX_STATE_IDLE_STABILIZATION_MS) {
    config->modbus_ct_idle_stabilization_ms =
        DEFAULT_MODBUS_CT_IDLE_STABILIZATION_MS;
  }
  config->automize_ct_part_enabled =
      nvs_read_u8(handle, NVS_KEY_AUTO_CT_PART_EN,
                  DEFAULT_AUTOMIZE_CT_PART_ENABLED ? 1 : 0) != 0;
  config->automize_ct_part_threshold_a =
      nvs_read_float(handle, NVS_KEY_AUTO_CT_PART_THR,
                     DEFAULT_AUTOMIZE_CT_PART_THRESHOLD_A);
  config->automize_ct_part_stabilization_ms =
      nvs_read_u32(handle, NVS_KEY_AUTO_CT_PART_STAB,
                   DEFAULT_AUTOMIZE_CT_PART_STABILIZATION_MS);

  // Load communication
  config->uplink_mode = (nvs_uplink_mode_t)nvs_read_u8(
      handle, NVS_KEY_UPLINK_MODE, (uint8_t)NVS_UPLINK_WIFI);
  config->mqtt_keepalive =
      nvs_read_u16(handle, NVS_KEY_MQTT_KEEPALIVE, DEFAULT_MQTT_KEEPALIVE);
  config->livedata_interval_ms =
      nvs_read_u32(handle, NVS_KEY_LIVEDATA_INT, DEFAULT_LIVEDATA_INT);
  nvs_read_string(handle, NVS_KEY_SSID_PREFIX, config->softap_ssid_prefix,
                  sizeof(config->softap_ssid_prefix), DEFAULT_SSID_PREFIX);

  // Load cloud config sync fields
  nvs_read_string(handle, NVS_KEY_CFG_ID, config->config_id,
                  NVS_CONFIG_ID_LEN, "");
  nvs_read_string(handle, NVS_KEY_CATALOG_VER, config->catalog_version,
                  NVS_CATALOG_VER_LEN, "");
  config->nominal_kw =
      nvs_read_float(handle, NVS_KEY_NOMINAL_KW, DEFAULT_NOMINAL_KW);
  config->kw_hysteresis_abs =
      nvs_read_float(handle, NVS_KEY_KW_HYST_ABS, DEFAULT_KW_HYST_ABS);
  config->expected_cadence_s =
      nvs_read_u32(handle, NVS_KEY_EXP_CADENCE, DEFAULT_EXPECTED_CADENCE);
  nvs_read_string(handle, NVS_KEY_SEC_UNIT, config->sec_unit,
                  sizeof(config->sec_unit), "unit");
  config->count_factor =
      nvs_read_float(handle, NVS_KEY_COUNT_FACTOR, DEFAULT_COUNT_FACTOR);
  config->ideal_cycle_time_sec =
      nvs_read_u32(handle, NVS_KEY_IDEAL_CYCLE, DEFAULT_IDEAL_CYCLE_SEC);
  nvs_read_string(handle, NVS_KEY_CT_POLARITY, config->ct_polarity,
                  sizeof(config->ct_polarity), "normal");
  config->pf_clamp = nvs_read_u8(handle, NVS_KEY_PF_CLAMP, 1) != 0;
  nvs_read_string(handle, NVS_KEY_SMOOTH_WIN, config->smoothing_window,
                  sizeof(config->smoothing_window), "off");

  // Load vibration config
  config->vib_enabled = nvs_read_u8(handle, NVS_KEY_VIB_EN, 1) != 0;
  config->vib_model = (nvs_vib_model_t)nvs_read_u8(
      handle, NVS_KEY_VIB_MODEL, (uint8_t)NVS_VIB_MODEL_VIBE_Q);
  if (config->vib_model != NVS_VIB_MODEL_RS_WZ3 &&
      config->vib_model != NVS_VIB_MODEL_WTVB01_485 &&
      config->vib_model != NVS_VIB_MODEL_VIBE_Q) {
    config->vib_model = NVS_VIB_MODEL_VIBE_Q;
  }
  config->vib_warning_mm_s =
      nvs_read_float(handle, NVS_KEY_VIB_WARN, DEFAULT_VIB_WARNING_MMS);
  config->vib_critical_mm_s =
      nvs_read_float(handle, NVS_KEY_VIB_CRIT, DEFAULT_VIB_CRITICAL_MMS);
  nvs_read_string(handle, NVS_KEY_VIB_SAMP, config->vib_sampling,
                  sizeof(config->vib_sampling), "normal");

  // Load system flags
  config->provisioned = nvs_read_u8(handle, NVS_KEY_PROVISIONED, 0) != 0;
  config->config_version = NVS_CONFIG_VERSION;  // FIX #13: always store current version
  config->boot_count = nvs_read_u32(handle, NVS_KEY_BOOT_COUNT, 0);

  nvs_close(handle);

  // Copy to internal state
  memcpy(&s_config, config, sizeof(nvs_device_config_t));
  s_config_loaded = true;
  publish_config_snapshot();

  ESP_LOGI(TAG, "Config loaded: device_id='%s', wifi_cfg=%d, provisioned=%d",
           config->device_id, config->wifi_configured, config->provisioned);

  UNLOCK();

  /*
   * FIX #13: Persist migrated config immediately on first boot after OTA.
   * Cannot call nvs_config_save() while holding LOCK (non-recursive mutex),
   * so we save here after releasing it. On the next boot saved_version ==
   * NVS_CONFIG_VERSION and this branch is not taken.
   */
  if (migration_needed) {
    ESP_LOGI(TAG, "[NVS_MIGRATION] Saving config at v%d", NVS_CONFIG_VERSION);
    nvs_config_save(config);
  }

  return ESP_OK;
}

esp_err_t nvs_config_save(const nvs_device_config_t *config) {
  if (!s_initialized) {
    ESP_LOGE(TAG, "NVS config not initialized");
    return ESP_ERR_INVALID_STATE;
  }
  if (config == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  LOCK();

  // Open NVS handle for writing
  nvs_handle_t handle;
  esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to open NVS for writing: %s", esp_err_to_name(ret));
    UNLOCK();
    return ret;
  }

  esp_err_t first_err = ESP_OK;
#define RECORD_NVS_OP(expr)                                                    \
  do {                                                                         \
    esp_err_t op_ret = (expr);                                                 \
    if (op_ret != ESP_OK && first_err == ESP_OK) {                             \
      first_err = op_ret;                                                      \
    }                                                                          \
  } while (0)

  // Save all fields
  RECORD_NVS_OP(nvs_write_string(handle, NVS_KEY_DEVICE_ID, config->device_id));
  RECORD_NVS_OP(nvs_write_string(handle, NVS_KEY_LOCATION, config->location));
  RECORD_NVS_OP(nvs_write_string(handle, NVS_KEY_WIFI_SSID, config->wifi_ssid));
  RECORD_NVS_OP(nvs_write_string(handle, NVS_KEY_WIFI_PASS, config->wifi_password));
  RECORD_NVS_OP(nvs_set_u8(handle, NVS_KEY_WIFI_CONFIGURED, config->wifi_configured ? 1 : 0));
  RECORD_NVS_OP(nvs_write_string(handle, NVS_KEY_MQTT_BROKER, config->mqtt_broker));
  RECORD_NVS_OP(nvs_set_u16(handle, NVS_KEY_MQTT_PORT, config->mqtt_port));
  RECORD_NVS_OP(nvs_write_string(handle, NVS_KEY_MQTT_USER, config->mqtt_username));
  RECORD_NVS_OP(nvs_write_string(handle, NVS_KEY_MQTT_PASS, config->mqtt_password));
  RECORD_NVS_OP(nvs_write_float(handle, NVS_KEY_CT_GAIN_0, config->ct_gain[0]));
  RECORD_NVS_OP(nvs_write_float(handle, NVS_KEY_CT_GAIN_1, config->ct_gain[1]));
  RECORD_NVS_OP(nvs_write_float(handle, NVS_KEY_CT_GAIN_2, config->ct_gain[2]));
  RECORD_NVS_OP(nvs_write_float(handle, NVS_KEY_ACTIVE_THR, config->active_threshold));
  RECORD_NVS_OP(nvs_write_float(handle, NVS_KEY_VOLTAGE_NOM, config->voltage_nominal));
  RECORD_NVS_OP(nvs_write_float(handle, NVS_KEY_POWER_FACTOR, config->power_factor));
  RECORD_NVS_OP(nvs_set_u8(handle, NVS_KEY_IGNORE_OPEN_CT, config->ignore_open_ct ? 1 : 0));
  RECORD_NVS_OP(nvs_write_float(handle, NVS_KEY_HYSTERESIS, config->hysteresis_factor));
  RECORD_NVS_OP(nvs_set_u32(handle, NVS_KEY_SUSTAIN_MS, config->active_sustain_ms));
  RECORD_NVS_OP(nvs_set_u32(handle, NVS_KEY_MACHINE_SETUP, config->machine_setup_time_ms));

  RECORD_NVS_OP(nvs_set_u8(handle, NVS_KEY_CT_MASK, config->ct_enabled_mask));
  RECORD_NVS_OP(nvs_set_u8(handle, NVS_KEY_CT_PHASE_MODE, (uint8_t)config->ct_phase_mode));
  RECORD_NVS_OP(nvs_set_u8(handle, NVS_KEY_STATE_CT_MODE, (uint8_t)config->state_ct_phase_mode));
  RECORD_NVS_OP(nvs_set_u8(handle, NVS_KEY_ENERGY_CT_MODE, (uint8_t)config->energy_ct_phase_mode));
  RECORD_NVS_OP(nvs_set_u32(handle, NVS_KEY_IDLE_STAB_MS, config->state_idle_stabilization_ms));
  RECORD_NVS_OP(nvs_set_u8(handle, NVS_KEY_PROXY_MASK, config->proxy_enabled_mask));
  RECORD_NVS_OP(nvs_set_u32(handle, NVS_KEY_PROXY_DEB_MS, config->proxy_debounce_ms));
  RECORD_NVS_OP(nvs_set_u8(handle, NVS_KEY_ENC_ENABLED, config->encoder_enabled ? 1 : 0));
  RECORD_NVS_OP(nvs_write_float(handle, NVS_KEY_ENC_PPR, config->encoder_ppr));
  RECORD_NVS_OP(nvs_write_float(handle, NVS_KEY_ENC_MPM_FAC, config->encoder_mpm_factor));
  RECORD_NVS_OP(nvs_set_u8(handle, NVS_KEY_ENC_MODE,
                           (uint8_t)config->encoder_mode));
  RECORD_NVS_OP(nvs_set_u8(handle, NVS_KEY_ENC_DIR,
                           (uint8_t)config->encoder_direction));
  RECORD_NVS_OP(nvs_write_float(handle, NVS_KEY_ENC_DIAMETER,
                                config->encoder_diameter_mm));
  RECORD_NVS_OP(nvs_write_float(handle, NVS_KEY_ENC_SCRAP_LEN,
                                config->encoder_scrap_length_m));
  RECORD_NVS_OP(nvs_set_u8(handle, NVS_KEY_ENC_SCRAP_EN,
                           config->encoder_scrap_enabled ? 1 : 0));
  RECORD_NVS_OP(nvs_set_u8(handle, NVS_KEY_STATE_SRC, (uint8_t)config->state_source));
  RECORD_NVS_OP(nvs_set_u8(handle, NVS_KEY_SIGNAL_LOGIC,
                           (uint8_t)config->signal_logic));
  RECORD_NVS_OP(nvs_set_u8(handle, NVS_KEY_PROXY_COUNT_EN,
                           config->proxy_counting_enabled ? 1 : 0));
  RECORD_NVS_OP(nvs_set_u8(handle, NVS_KEY_PROXY_COUNT_MODE,
                           (uint8_t)config->proxy_count_mode));
  RECORD_NVS_OP(nvs_write_float(handle, NVS_KEY_PROXY_METER_COUNT,
                                config->proxy_meter_per_count));
  RECORD_NVS_OP(nvs_write_float(handle, NVS_KEY_PROXY_SCRAP_CYCLE,
                                 config->proxy_scrap_cycle_time_s));
  RECORD_NVS_OP(nvs_set_u8(handle, NVS_KEY_PROXY_SCRAP_EN,
                           config->proxy_scrap_enabled ? 1 : 0));
  RECORD_NVS_OP(nvs_write_float(handle, NVS_KEY_CYCLE_CNT_THR_A,
                                config->cycle_time_count_threshold_a));
  RECORD_NVS_OP(nvs_write_float(handle, NVS_KEY_CYCLE_CNT_HYST_A,
                                config->cycle_time_count_hysteresis_a));
  RECORD_NVS_OP(nvs_set_u8(handle, NVS_KEY_CYCLE_CNT_SRC,
                           (uint8_t)config->cycle_count_source));
  RECORD_NVS_OP(nvs_set_u8(handle, NVS_KEY_CYCLE_SIG_LOGIC,
                           (uint8_t)config->cycle_signal_logic));
  RECORD_NVS_OP(nvs_set_u8(handle, NVS_KEY_CYCLE_SIG_LOGIC_A,
                           (uint8_t)config->cycle_signal_logic_a));
  RECORD_NVS_OP(nvs_set_u8(handle, NVS_KEY_CYCLE_SIG_LOGIC_B,
                           (uint8_t)config->cycle_signal_logic_b));
  RECORD_NVS_OP(nvs_set_u32(handle, NVS_KEY_CYCLE_SIG_REARM,
                            config->cycle_signal_rearm_ms));
  RECORD_NVS_OP(nvs_set_u8(handle, NVS_KEY_SPEED_ENABLED,
                           config->speed_enabled ? 1 : 0));
  RECORD_NVS_OP(nvs_write_float(handle, NVS_KEY_SPEED_THR_A,
                                config->speed_threshold_a));
  RECORD_NVS_OP(nvs_write_float(handle, NVS_KEY_SPEED_MPS,
                                config->speed_mps));
  RECORD_NVS_OP(nvs_set_u8(handle, NVS_KEY_SCRAP_SOURCE,
                           (uint8_t)config->scrap_source));
  RECORD_NVS_OP(nvs_set_u8(handle, NVS_KEY_MODBUS_CT_EN,
                           config->modbus_ct_enabled ? 1 : 0));
  RECORD_NVS_OP(nvs_set_u8(handle, NVS_KEY_MODBUS_CT_MODEL,
                           (uint8_t)config->modbus_ct_model));
  RECORD_NVS_OP(nvs_set_u8(handle, NVS_KEY_MODBUS_CT_NET,
                           (uint8_t)config->modbus_ct_network));
  RECORD_NVS_OP(nvs_write_float(handle, NVS_KEY_MODBUS_CT_THR,
                                config->modbus_ct_threshold_kva));
  RECORD_NVS_OP(nvs_write_float(handle, NVS_KEY_MODBUS_CT_HYST,
                                config->modbus_ct_hysteresis_factor));
  RECORD_NVS_OP(nvs_set_u32(handle, NVS_KEY_MODBUS_CT_IDLE,
                            config->modbus_ct_idle_stabilization_ms));
  RECORD_NVS_OP(nvs_set_u8(handle, NVS_KEY_AUTO_CT_PART_EN,
                           config->automize_ct_part_enabled ? 1 : 0));
  RECORD_NVS_OP(nvs_write_float(handle, NVS_KEY_AUTO_CT_PART_THR,
                                config->automize_ct_part_threshold_a));
  RECORD_NVS_OP(nvs_set_u32(handle, NVS_KEY_AUTO_CT_PART_STAB,
                            config->automize_ct_part_stabilization_ms));

  RECORD_NVS_OP(nvs_set_u8(handle, NVS_KEY_UPLINK_MODE, (uint8_t)config->uplink_mode));
  RECORD_NVS_OP(nvs_set_u16(handle, NVS_KEY_MQTT_KEEPALIVE, config->mqtt_keepalive));
  RECORD_NVS_OP(nvs_set_u32(handle, NVS_KEY_LIVEDATA_INT, config->livedata_interval_ms));
  RECORD_NVS_OP(nvs_write_string(handle, NVS_KEY_SSID_PREFIX, config->softap_ssid_prefix));
  // Save cloud config sync fields
  RECORD_NVS_OP(nvs_write_string(handle, NVS_KEY_CFG_ID, config->config_id));
  RECORD_NVS_OP(nvs_write_string(handle, NVS_KEY_CATALOG_VER, config->catalog_version));
  RECORD_NVS_OP(nvs_write_float(handle, NVS_KEY_NOMINAL_KW, config->nominal_kw));
  RECORD_NVS_OP(nvs_write_float(handle, NVS_KEY_KW_HYST_ABS, config->kw_hysteresis_abs));
  RECORD_NVS_OP(nvs_set_u32(handle, NVS_KEY_EXP_CADENCE, config->expected_cadence_s));
  RECORD_NVS_OP(nvs_write_string(handle, NVS_KEY_SEC_UNIT, config->sec_unit));
  RECORD_NVS_OP(nvs_write_float(handle, NVS_KEY_COUNT_FACTOR, config->count_factor));
  RECORD_NVS_OP(nvs_set_u32(handle, NVS_KEY_IDEAL_CYCLE, config->ideal_cycle_time_sec));
  RECORD_NVS_OP(nvs_write_string(handle, NVS_KEY_CT_POLARITY, config->ct_polarity));
  RECORD_NVS_OP(nvs_set_u8(handle, NVS_KEY_PF_CLAMP, config->pf_clamp ? 1 : 0));
  RECORD_NVS_OP(nvs_write_string(handle, NVS_KEY_SMOOTH_WIN, config->smoothing_window));

  // Save vibration config
  RECORD_NVS_OP(nvs_set_u8(handle, NVS_KEY_VIB_EN, config->vib_enabled ? 1 : 0));
  RECORD_NVS_OP(nvs_set_u8(handle, NVS_KEY_VIB_MODEL, (uint8_t)config->vib_model));
  RECORD_NVS_OP(nvs_write_float(handle, NVS_KEY_VIB_WARN, config->vib_warning_mm_s));
  RECORD_NVS_OP(nvs_write_float(handle, NVS_KEY_VIB_CRIT, config->vib_critical_mm_s));
  RECORD_NVS_OP(nvs_write_string(handle, NVS_KEY_VIB_SAMP, config->vib_sampling));

  RECORD_NVS_OP(nvs_set_u8(handle, NVS_KEY_PROVISIONED, config->provisioned ? 1 : 0));
  RECORD_NVS_OP(nvs_set_u32(handle, NVS_KEY_CONFIG_VER, NVS_CONFIG_VERSION));
  RECORD_NVS_OP(nvs_set_u32(handle, NVS_KEY_BOOT_COUNT, config->boot_count));

  if (first_err != ESP_OK) {
    ESP_LOGE(TAG, "Config save aborted before commit: %s", esp_err_to_name(first_err));
    nvs_close(handle);
    UNLOCK();
    return first_err;
  }
#undef RECORD_NVS_OP

  // Commit changes
  ret = nvs_commit(handle);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to commit NVS: %s", esp_err_to_name(ret));
  } else {
    ESP_LOGI(TAG, "Config saved successfully");
  }

  nvs_close(handle);

  // Update internal state only after flash commit succeeds.
  if (ret == ESP_OK) {
    memcpy(&s_config, config, sizeof(nvs_device_config_t));
    s_config_loaded = true;
  publish_config_snapshot();
  }

  UNLOCK();
  return ret;
}

esp_err_t nvs_config_factory_reset(void) {
  if (!s_initialized) {
    ESP_LOGE(TAG, "NVS config not initialized");
    return ESP_ERR_INVALID_STATE;
  }

  LOCK();

  // Erase the namespace
  nvs_handle_t handle;
  esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
  if (ret == ESP_OK) {
    ret = nvs_erase_all(handle);
    if (ret == ESP_OK) {
      ret = nvs_commit(handle);
    }
    nvs_close(handle);
  }

  // Reset internal state to defaults
  nvs_config_load_defaults(&s_config);
  s_config_loaded = true;
  publish_config_snapshot();

  ESP_LOGI(TAG, "Factory reset complete");

  UNLOCK();
  return ret;
}

bool nvs_config_is_provisioned(void) {
  if (!s_config_loaded)
    return false;
  return s_config.provisioned;
}

bool nvs_config_is_wifi_configured(void) {
  if (!s_config_loaded)
    return false;
  return s_config.wifi_configured && strlen(s_config.wifi_ssid) > 0;
}

const nvs_device_config_t *nvs_config_get(void) {
  if (!s_config_loaded)
    return NULL;
  return &s_config;
}

bool nvs_config_snapshot(nvs_device_config_t *out) {
  if (!out) return false;
  portENTER_CRITICAL(&s_snapshot_mux);
  bool valid = s_read_snapshot_valid;
  if (valid) *out = s_read_snapshot;
  portEXIT_CRITICAL(&s_snapshot_mux);
  return valid;
}

esp_err_t nvs_config_set_string(const char *key, const char *value) {
  if (!s_initialized || key == NULL || value == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  LOCK();

  nvs_handle_t handle;
  esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
  if (ret != ESP_OK) {
    UNLOCK();
    return ret;
  }

  ret = nvs_write_string(handle, key, value);
  if (ret == ESP_OK) {
    ret = nvs_commit(handle);
  }
  nvs_close(handle);

  UNLOCK();
  return ret;
}

void nvs_config_get_mac_string(char *mac_out, size_t len) {
  if (mac_out == NULL || len < 18) {
    return;
  }

  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_WIFI_STA);
  snprintf(mac_out, len, "%02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1],
           mac[2], mac[3], mac[4], mac[5]);
}

esp_err_t nvs_config_write_reason_catalog(const char *json_str) {
  if (!s_initialized || json_str == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  size_t len = strlen(json_str);
  if (len >= NVS_REASON_CATALOG_MAX_LEN) {
    ESP_LOGE(TAG, "Reason catalog too large: %d bytes", (int)len);
    return ESP_ERR_INVALID_SIZE;
  }

  LOCK();

  nvs_handle_t handle;
  esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
  if (ret != ESP_OK) {
    UNLOCK();
    return ret;
  }

  ret = nvs_set_blob(handle, NVS_KEY_REASON_CAT, json_str, len + 1);
  if (ret == ESP_OK) {
    ret = nvs_commit(handle);
  }

  nvs_close(handle);
  UNLOCK();

  if (ret == ESP_OK) {
    ESP_LOGI(TAG, "Reason catalog saved (%d bytes)", (int)len);
  } else {
    ESP_LOGE(TAG, "Failed to save reason catalog: %s", esp_err_to_name(ret));
  }

  return ret;
}

esp_err_t nvs_config_read_reason_catalog(char *buf, size_t buf_len) {
  if (!s_initialized || buf == NULL || buf_len == 0) {
    return ESP_ERR_INVALID_ARG;
  }

  LOCK();

  nvs_handle_t handle;
  esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);
  if (ret != ESP_OK) {
    UNLOCK();
    return ret;
  }

  size_t actual_len = buf_len;
  ret = nvs_get_blob(handle, NVS_KEY_REASON_CAT, buf, &actual_len);
  if (ret == ESP_ERR_NVS_NOT_FOUND) {
    buf[0] = '\0';
  } else if (ret != ESP_OK) {
    ESP_LOGW(TAG, "Failed to read reason catalog: %s", esp_err_to_name(ret));
    buf[0] = '\0';
  }

  nvs_close(handle);
  UNLOCK();

  return ret;
}

// ============================================================================
// Stoppage Reason Persistence (survives reboot)
// ============================================================================

esp_err_t nvs_config_save_stoppage(uint8_t code, const char *l1,
                                    const char *l2) {
  if (!s_initialized)
    return ESP_ERR_INVALID_STATE;

  LOCK();

  nvs_handle_t handle;
  esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
  if (ret != ESP_OK) {
    UNLOCK();
    return ret;
  }

  ret = nvs_set_u8(handle, NVS_KEY_STP_CODE, code);
  if (ret == ESP_OK) ret = nvs_write_string(handle, NVS_KEY_STP_L1, l1 ? l1 : "");
  if (ret == ESP_OK) ret = nvs_write_string(handle, NVS_KEY_STP_L2, l2 ? l2 : "");
  if (ret == ESP_OK) ret = nvs_commit(handle);

  nvs_close(handle);
  UNLOCK();

  if (ret == ESP_OK) {
    ESP_LOGI(TAG, "Stoppage saved: code=%d", code);
  }
  return ret;
}

esp_err_t nvs_config_load_stoppage(uint8_t *code, char *l1, size_t l1_len,
                                    char *l2, size_t l2_len) {
  if (!s_initialized || code == NULL)
    return ESP_ERR_INVALID_STATE;

  LOCK();

  nvs_handle_t handle;
  esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);
  if (ret != ESP_OK) {
    UNLOCK();
    return ret;
  }

  uint8_t saved_code = 0;
  ret = nvs_get_u8(handle, NVS_KEY_STP_CODE, &saved_code);
  if (ret == ESP_ERR_NVS_NOT_FOUND) {
    nvs_close(handle);
    UNLOCK();
    return ESP_ERR_NOT_FOUND;
  }

  *code = saved_code;
  if (l1)
    nvs_read_string(handle, NVS_KEY_STP_L1, l1, l1_len, "");
  if (l2)
    nvs_read_string(handle, NVS_KEY_STP_L2, l2, l2_len, "");

  nvs_close(handle);
  UNLOCK();

  ESP_LOGI(TAG, "Stoppage loaded: code=%d", saved_code);
  return ESP_OK;
}

esp_err_t nvs_config_clear_stoppage(void) {
  if (!s_initialized)
    return ESP_ERR_INVALID_STATE;

  LOCK();

  nvs_handle_t handle;
  esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
  if (ret != ESP_OK) {
    UNLOCK();
    return ret;
  }

  ret = nvs_erase_key(handle, NVS_KEY_STP_CODE);
  if (ret == ESP_ERR_NVS_NOT_FOUND) ret = ESP_OK;
  if (ret == ESP_OK) {
    esp_err_t erase_ret = nvs_erase_key(handle, NVS_KEY_STP_L1);
    if (erase_ret != ESP_OK && erase_ret != ESP_ERR_NVS_NOT_FOUND) ret = erase_ret;
  }
  if (ret == ESP_OK) {
    esp_err_t erase_ret = nvs_erase_key(handle, NVS_KEY_STP_L2);
    if (erase_ret != ESP_OK && erase_ret != ESP_ERR_NVS_NOT_FOUND) ret = erase_ret;
  }
  if (ret == ESP_OK) ret = nvs_commit(handle);

  nvs_close(handle);
  UNLOCK();

  ESP_LOGD(TAG, "Stoppage cleared from NVS");
  return ret;
}

esp_err_t nvs_config_load_energy(float *today_kwh, float *total_kwh,
                                 int32_t *day_key) {
  if (!s_initialized || today_kwh == NULL || total_kwh == NULL ||
      day_key == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  LOCK();

  nvs_handle_t handle;
  esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);
  if (ret != ESP_OK) {
    UNLOCK();
    return ret;
  }

  float today = 0.0f;
  float total = 0.0f;
  int32_t stored_day = -1;
  size_t len = sizeof(float);
  ret = nvs_get_blob(handle, NVS_KEY_ENERGY_TODAY, &today, &len);
  if (ret == ESP_OK) {
    len = sizeof(float);
    ret = nvs_get_blob(handle, NVS_KEY_ENERGY_TOTAL, &total, &len);
  }
  if (ret == ESP_OK) {
    ret = nvs_get_i32(handle, NVS_KEY_ENERGY_DAY, &stored_day);
    if (ret == ESP_ERR_NVS_NOT_FOUND) {
      stored_day = -1;
      ret = ESP_OK;
    }
  }

  nvs_close(handle);
  UNLOCK();

  if (ret == ESP_OK) {
    *today_kwh = today;
    *total_kwh = total;
    *day_key = stored_day;
    ESP_LOGI(TAG, "Energy counters loaded: today=%.4f total=%.4f day=%ld",
             today, total, (long)stored_day);
  }

  return ret;
}

esp_err_t nvs_config_save_energy(float today_kwh, float total_kwh,
                                 int32_t day_key) {
  if (!s_initialized) {
    return ESP_ERR_INVALID_STATE;
  }

  LOCK();

  nvs_handle_t handle;
  esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
  if (ret != ESP_OK) {
    UNLOCK();
    return ret;
  }

  ret = nvs_set_blob(handle, NVS_KEY_ENERGY_TODAY, &today_kwh,
                     sizeof(today_kwh));
  if (ret == ESP_OK) {
    ret = nvs_set_blob(handle, NVS_KEY_ENERGY_TOTAL, &total_kwh,
                       sizeof(total_kwh));
  }
  if (ret == ESP_OK) {
    ret = nvs_set_i32(handle, NVS_KEY_ENERGY_DAY, day_key);
  }
  if (ret == ESP_OK) {
    ret = nvs_commit(handle);
  }

  nvs_close(handle);
  UNLOCK();

  if (ret != ESP_OK) {
    ESP_LOGW(TAG, "Energy counter save failed: %s", esp_err_to_name(ret));
  }
  return ret;
}

esp_err_t nvs_config_load_speed_total(float *total_length_m) {
  if (!s_initialized || total_length_m == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  LOCK();

  nvs_handle_t handle;
  esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);
  if (ret != ESP_OK) {
    UNLOCK();
    return ret;
  }

  float total = 0.0f;
  size_t len = sizeof(total);
  ret = nvs_get_blob(handle, NVS_KEY_SPEED_TOTAL, &total, &len);

  nvs_close(handle);
  UNLOCK();

  if (ret == ESP_OK) {
    if (!isfinite(total) || total < 0.0f) {
      total = 0.0f;
    }
    *total_length_m = total;
    ESP_LOGI(TAG, "Speed total loaded: %.3fm", total);
  }
  return ret;
}

esp_err_t nvs_config_save_speed_total(float total_length_m) {
  if (!s_initialized) {
    return ESP_ERR_INVALID_STATE;
  }
  if (!isfinite(total_length_m) || total_length_m < 0.0f) {
    total_length_m = 0.0f;
  }

  LOCK();

  nvs_handle_t handle;
  esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
  if (ret != ESP_OK) {
    UNLOCK();
    return ret;
  }

  ret = nvs_set_blob(handle, NVS_KEY_SPEED_TOTAL, &total_length_m,
                     sizeof(total_length_m));
  if (ret == ESP_OK) {
    ret = nvs_commit(handle);
  }

  nvs_close(handle);
  UNLOCK();

  if (ret != ESP_OK) {
    ESP_LOGW(TAG, "Speed total save failed: %s", esp_err_to_name(ret));
  }
  return ret;
}

esp_err_t nvs_config_load_common_product_count(uint64_t *product_count) {
  if (!s_initialized || product_count == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  *product_count = 0;
  LOCK();
  nvs_handle_t handle;
  esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);
  if (ret == ESP_OK) {
    ret = nvs_get_u64(handle, NVS_KEY_COMMON_PROD_COUNT, product_count);
    nvs_close(handle);
  }
  UNLOCK();
  return ret;
}

esp_err_t nvs_config_save_common_product_count(uint64_t product_count) {
  if (!s_initialized) return ESP_ERR_INVALID_STATE;
  LOCK();
  nvs_handle_t handle;
  esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
  if (ret == ESP_OK) {
    ret = nvs_set_u64(handle, NVS_KEY_COMMON_PROD_COUNT, product_count);
    if (ret == ESP_OK) ret = nvs_commit(handle);
    nvs_close(handle);
  }
  UNLOCK();
  return ret;
}

esp_err_t nvs_config_save_energy_reset_event(const char *reason,
                                             float pre_reset_value,
                                             const char *timestamp) {
  if (!s_initialized || reason == NULL || timestamp == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  LOCK();

  nvs_handle_t handle;
  esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
  bool opened = (ret == ESP_OK);
  if (ret == ESP_OK) {
    ret = nvs_write_string(handle, NVS_KEY_ENERGY_RST_REASON, reason);
  }
  if (ret == ESP_OK) {
    ret = nvs_set_blob(handle, NVS_KEY_ENERGY_RST_VALUE, &pre_reset_value,
                       sizeof(pre_reset_value));
  }
  if (ret == ESP_OK) {
    ret = nvs_write_string(handle, NVS_KEY_ENERGY_RST_TS, timestamp);
  }
  if (ret == ESP_OK) {
    ret = nvs_commit(handle);
  }

  if (opened) {
    nvs_close(handle);
  }
  UNLOCK();

  if (ret == ESP_OK) {
    ESP_LOGI(TAG, "Energy reset event saved: %s pre=%.4f ts=%s", reason,
             pre_reset_value, timestamp);
  } else {
    ESP_LOGW(TAG, "Energy reset event save failed: %s", esp_err_to_name(ret));
  }
  return ret;
}

void nvs_duration_load_defaults(nvs_duration_store_t *store) {
  if (store == NULL) {
    return;
  }
  memset(store, 0, sizeof(*store));
  store->magic = NVS_DURATION_MAGIC;
  store->version = NVS_DURATION_VERSION;
}

static void nvs_duration_sanitize(nvs_duration_store_t *store) {
  if (store == NULL) {
    return;
  }

  store->magic = NVS_DURATION_MAGIC;
  store->version = NVS_DURATION_VERSION;
  if (!store->selected_valid || store->selected_part_number == 0 ||
      store->cycle_time_s == 0 || store->operations_per_part == 0) {
    store->selected_valid = 0;
    store->selected_part_number = 0;
    store->cycle_time_s = 0;
    store->operations_per_part = 0;
  }

  store->signal_enabled_mask &= 0x03;
  for (uint8_t i = 0; i < NVS_SIGNAL_COUNT_CHANNELS; i++) {
    if (!store->signal_selected_valid[i] ||
        store->signal_part_number[i] == 0 ||
        store->signal_cycle_time_s[i] == 0 ||
        store->signal_operations_per_part[i] == 0) {
      store->signal_selected_valid[i] = 0;
      store->signal_part_number[i] = 0;
      store->signal_cycle_time_s[i] = 0;
      store->signal_operations_per_part[i] = 0;
    }
  }
}

esp_err_t nvs_config_load_duration(nvs_duration_store_t *store) {
  if (!s_initialized || store == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  nvs_duration_load_defaults(store);
  LOCK();

  nvs_handle_t handle;
  esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);
  if (ret != ESP_OK) {
    UNLOCK();
    return ret;
  }

  size_t len = sizeof(*store);
  ret = nvs_get_blob(handle, NVS_KEY_DURATION_STORE, store, &len);
  nvs_close(handle);
  UNLOCK();

  if (ret == ESP_ERR_NVS_NOT_FOUND) {
    nvs_duration_load_defaults(store);
    return ret;
  }
  if (ret == ESP_OK && len == sizeof(nvs_duration_store_v2_t)) {
    nvs_duration_store_v2_t legacy;
    memcpy(&legacy, store, sizeof(legacy));
    if (legacy.magic != NVS_DURATION_MAGIC || legacy.version != 2) {
      ESP_LOGW(TAG, "Legacy duration store invalid, using defaults");
      nvs_duration_load_defaults(store);
      return ESP_ERR_INVALID_SIZE;
    }
    nvs_duration_load_defaults(store);
    store->count_enabled = legacy.count_enabled;
    store->selected_valid = legacy.selected_valid;
    store->selected_part_number = legacy.selected_part_number;
    store->finished_count = legacy.finished_count;
    store->cycle_time_s = legacy.cycle_time_s;
    store->operations_per_part = legacy.operations_per_part;
    nvs_duration_sanitize(store);
    return ESP_OK;
  }
  if (ret != ESP_OK || len != sizeof(*store) ||
      store->magic != NVS_DURATION_MAGIC ||
      store->version != NVS_DURATION_VERSION) {
    ESP_LOGW(TAG, "Duration store invalid, using defaults");
    nvs_duration_load_defaults(store);
    return (ret == ESP_OK) ? ESP_ERR_INVALID_SIZE : ret;
  }

  nvs_duration_sanitize(store);
  return ESP_OK;
}

esp_err_t nvs_config_save_duration(const nvs_duration_store_t *store) {
  if (!s_initialized || store == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  nvs_duration_store_t clean;
  memcpy(&clean, store, sizeof(clean));
  nvs_duration_sanitize(&clean);

  LOCK();
  nvs_handle_t handle;
  esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
  if (ret != ESP_OK) {
    UNLOCK();
    return ret;
  }

  ret = nvs_set_blob(handle, NVS_KEY_DURATION_STORE, &clean, sizeof(clean));
  if (ret == ESP_OK) {
    ret = nvs_commit(handle);
  }
  nvs_close(handle);
  UNLOCK();

  if (ret != ESP_OK) {
    ESP_LOGW(TAG, "Duration store save failed: %s", esp_err_to_name(ret));
  }
  return ret;
}
