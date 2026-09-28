/**
 * @file json_data_logger.c
 * @brief JSON Data Logger implementation for CT+HMI System
 *
 * Uses ESP-IDF's built-in cJSON library for JSON generation.
 * Outputs to serial log with MQTT topic format for future integration.
 */

#include "json_data_logger.h"
#include "cJSON.h"
#include "log_config.h"
#undef LOG_LOCAL_LEVEL
#define LOG_LOCAL_LEVEL LOG_LEVEL_JSON_LOGGER
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_config.h"
#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include <string.h>
#include <sys/time.h>
#include <time.h>

static const char *TAG = "json_logger";

// State name strings for JSON output
static const char *JSON_STATE_NAMES[] = {"RUN", "IDLE", "STP"};

// Logger configuration (stored after init)
static json_logger_config_t s_config = {0};
static bool s_initialized = false;

// Periodic livedata tracking
static uint32_t s_last_livedata_ms = 0;

// Periodic health tracking (30s interval)
#define HEALTH_INTERVAL_MS 30000
static uint32_t s_last_health_ms = 0;

// Sequence counter for statechange topic messages (state + reason).
// Incremented on every state_change or reason_code publish; allows cloud
// to deduplicate MQTT QoS-1 retries and detect dropped messages. (#20)
static uint32_t s_state_seq = 0;

// Last-known statechange and reason JSON cached for reconnect flush.
// Cleared/replaced on every new publish so flush always sends current state.
#define LAST_STATE_JSON_LEN  256
#define LAST_REASON_JSON_LEN 512
static char s_last_state_json[LAST_STATE_JSON_LEN]   = {0};
static char s_last_reason_json[LAST_REASON_JSON_LEN] = {0};

// MQTT callback function pointers
static mqtt_publish_fn_t s_mqtt_state_cb = NULL;
static mqtt_publish_fn_t s_mqtt_livedata_cb = NULL;
/* A successful submit means queue admission, not a broker acknowledgement.
 * Transport functions may wait seconds; only this worker invokes them. */
#define JSON_TX_DEPTH 12
#define JSON_TX_MAX_PAYLOAD 16384
typedef struct {
  char topic[96];
  char *payload;
  bool state;
  uint32_t generation;
} json_tx_job_t;
static QueueHandle_t s_tx_queue;
static portMUX_TYPE s_tx_lock = portMUX_INITIALIZER_UNLOCKED;
static uint32_t s_tx_generation, s_tx_dropped, s_tx_failed, s_tx_completed;

static void json_tx_worker(void *arg) {
  json_tx_job_t job;
  while (1) {
    if (xQueueReceive(s_tx_queue, &job, portMAX_DELAY) != pdTRUE) continue;
    portENTER_CRITICAL(&s_tx_lock);
    mqtt_publish_fn_t cb = job.state ? s_mqtt_state_cb : s_mqtt_livedata_cb;
    bool current = job.generation == s_tx_generation;
    portEXIT_CRITICAL(&s_tx_lock);
    esp_err_t ret = current && cb ? cb(job.topic, job.payload) : ESP_ERR_INVALID_STATE;
    free(job.payload);
    portENTER_CRITICAL(&s_tx_lock);
    if (ret == ESP_OK) s_tx_completed++;
    else s_tx_failed++;
    portEXIT_CRITICAL(&s_tx_lock);
    if (ret != ESP_OK) ESP_LOGW(TAG, "Async MQTT send failed: %s", esp_err_to_name(ret));
  }
}

static esp_err_t json_tx_submit(bool state, const char *topic, const char *payload) {
  if (!s_tx_queue || !topic || !payload) return ESP_ERR_INVALID_STATE;
  size_t len = strlen(payload);
  if (strlen(topic) >= sizeof(((json_tx_job_t *)0)->topic) || len > JSON_TX_MAX_PAYLOAD)
    return ESP_ERR_INVALID_SIZE;
  json_tx_job_t job = {.state = state};
  portENTER_CRITICAL(&s_tx_lock);
  job.generation = s_tx_generation;
  bool available = state ? s_mqtt_state_cb != NULL : s_mqtt_livedata_cb != NULL;
  portEXIT_CRITICAL(&s_tx_lock);
  if (!available) return ESP_ERR_INVALID_STATE;
  job.payload = malloc(len + 1);
  if (job.payload) {
    memcpy(job.payload, payload, len + 1);
    memcpy(job.topic, topic, strlen(topic) + 1);
    if (xQueueSend(s_tx_queue, &job, 0) == pdTRUE) return ESP_OK;
    free(job.payload);
  }
  portENTER_CRITICAL(&s_tx_lock);
  s_tx_dropped++;
  portEXIT_CRITICAL(&s_tx_lock);
  return ESP_ERR_NO_MEM;
}


/**
 * @brief Get current uptime in centiseconds
 */
static uint64_t get_uptime_cs(void) {
  return esp_timer_get_time() / 10000; // microseconds to centiseconds
}

/**
 * @brief Generate timestamp string
 *
 * If system time is valid (after 2020), uses ISO 8601 format.
 * Otherwise uses uptime format: UPTIME:HH:MM:SS.cc
 *
 * @param buf Output buffer (must be at least 32 bytes)
 * @param len Buffer length
 */
static void generate_timestamp(char *buf, size_t len) {
  time_t now;
  struct tm timeinfo;
  time(&now);
  localtime_r(&now, &timeinfo);

  // Check if system time is valid (year > 2020)
  if (timeinfo.tm_year > (2020 - 1900)) {
    // Valid system time - use ISO 8601 format
    strftime(buf, len, "%Y-%m-%dT%H:%M:%S", &timeinfo);
  } else {
    // No valid time - use uptime format
    uint64_t uptime_cs = get_uptime_cs();
    uint32_t total_seconds = (uint32_t)(uptime_cs / 100);
    uint32_t centiseconds = (uint32_t)(uptime_cs % 100);
    uint32_t hours = total_seconds / 3600;
    uint32_t minutes = (total_seconds % 3600) / 60;
    uint32_t seconds = total_seconds % 60;

    snprintf(buf, len, "UPTIME:%02lu:%02lu:%02lu.%02lu", (unsigned long)hours,
             (unsigned long)minutes, (unsigned long)seconds,
             (unsigned long)centiseconds);
  }
}

/**
 * @brief Round float to 2 decimal places
 */
static double round_2dp(float value) {
  return ((int)(value * 100.0f + 0.5f)) / 100.0;
}

/**
 * @brief Get current time in milliseconds
 */
static uint32_t get_time_ms(void) {
  return (uint32_t)(esp_timer_get_time() / 1000);
}

/**
 * @brief Build MQTT topic string
 *
 * @param buf Output buffer
 * @param len Buffer length
 * @param suffix Topic suffix (e.g., "statechange", "livedata")
 */
static void build_topic(char *buf, size_t len, const char *suffix) {
  snprintf(buf, len, "Limelight/factory/%s/%s", s_config.client_id, suffix);
}

esp_err_t json_logger_init(const json_logger_config_t *config) {
  // If config is NULL, load from NVS
  if (config == NULL) {
    nvs_device_config_t nvs_snap;
    const nvs_device_config_t *nvs_cfg = nvs_config_snapshot(&nvs_snap) ? &nvs_snap : NULL;
    if (nvs_cfg != NULL) {
      strncpy(s_config.client_id, nvs_cfg->device_id,
              sizeof(s_config.client_id) - 1);
    }
    s_config.livedata_interval_ms = nvs_cfg ? nvs_cfg->livedata_interval_ms : DEFAULT_LIVEDATA_INT;
    s_config.log_to_serial = true;
    ESP_LOGI(TAG, "JSON Logger config loaded from NVS");
  } else {
    // Copy configuration from parameter
    memcpy(&s_config, config, sizeof(json_logger_config_t));
  }

  // Ensure null-terminated client_id
  s_config.client_id[sizeof(s_config.client_id) - 1] = '\0';

  // Default interval if not set
  if (s_config.livedata_interval_ms == 0) {
    s_config.livedata_interval_ms = 5000;
  }

  if (!s_tx_queue) {
    s_tx_queue = xQueueCreate(JSON_TX_DEPTH, sizeof(json_tx_job_t));
    if (!s_tx_queue) return ESP_ERR_NO_MEM;
    if (xTaskCreatePinnedToCore(json_tx_worker, "json_tx", 6144, NULL, 2, NULL, 0) != pdPASS) {
      vQueueDelete(s_tx_queue);
      s_tx_queue = NULL;
      return ESP_ERR_NO_MEM;
    }
  }
  s_initialized = true;
  s_last_livedata_ms = get_time_ms();

  ESP_LOGI(TAG,
           "JSON Logger initialized: client_id=%s, interval=%lums, serial=%d",
           s_config.client_id, (unsigned long)s_config.livedata_interval_ms,
           s_config.log_to_serial ? 1 : 0);

  return ESP_OK;
}

void json_logger_set_mqtt_callback(mqtt_publish_fn_t state_cb,
                                   mqtt_publish_fn_t livedata_cb) {
  portENTER_CRITICAL(&s_tx_lock);
  s_tx_generation++;
  s_mqtt_state_cb = state_cb;
  s_mqtt_livedata_cb = livedata_cb;
  portEXIT_CRITICAL(&s_tx_lock);
  ESP_LOGI(TAG, "MQTT callbacks %s",
           (state_cb || livedata_cb) ? "registered" : "cleared");
}

bool json_logger_mqtt_available(void) {
  return (s_mqtt_state_cb != NULL || s_mqtt_livedata_cb != NULL);
}

void json_logger_flush_pending(void) {
  if (s_mqtt_state_cb == NULL) return;

  char topic[64];
  build_topic(topic, sizeof(topic), "statechange");

  if (s_last_state_json[0] != '\0') {
    esp_err_t ret = json_tx_submit(true, topic, s_last_state_json);
    ESP_LOGI(TAG, "[FLUSH] statechange: %s", ret == ESP_OK ? "OK" : "FAIL");
  }

  if (s_last_reason_json[0] != '\0') {
    esp_err_t ret = json_tx_submit(true, topic, s_last_reason_json);
    ESP_LOGI(TAG, "[FLUSH] reason: %s", ret == ESP_OK ? "OK" : "FAIL");
  }
}

void json_logger_state_change(json_machine_state_t new_state) {
  if (!s_initialized) {
    return;
  }

  if (new_state > JSON_STATE_STP) {
    new_state = JSON_STATE_ACT;
  }

  // Build JSON
  cJSON *root = cJSON_CreateObject();
  if (root == NULL) {
    ESP_LOGE(TAG, "Failed to create JSON object");
    return;
  }

  cJSON_AddStringToObject(root, "type", "state");
  cJSON_AddStringToObject(root, "client_id", s_config.client_id);
  cJSON_AddStringToObject(root, "state", JSON_STATE_NAMES[new_state]);
  nvs_device_config_t nvs_snap;
  const nvs_device_config_t *nvs_cfg =
      nvs_config_snapshot(&nvs_snap) ? &nvs_snap : NULL;
  bool signal_based =
      nvs_cfg && nvs_cfg->state_source == NVS_STATE_SRC_SIGNAL;
  cJSON_AddStringToObject(root, "state_detection",
                          signal_based ? "signal_based" : "ct");
  if (signal_based) {
    cJSON_AddStringToObject(
        root, "signal_logic",
        (nvs_cfg->signal_logic == NVS_SIGNAL_LOGIC_NC) ? "NC" : "NO");
  }
  cJSON_AddNumberToObject(root, "seq", (double)(++s_state_seq));

  // Generate JSON string
  char *json_str = cJSON_PrintUnformatted(root);
  if (json_str != NULL) {
    char topic[64];
    build_topic(topic, sizeof(topic), "statechange");

    // Cache for reconnect flush; new state supersedes previous reason
    strncpy(s_last_state_json, json_str, LAST_STATE_JSON_LEN - 1);
    s_last_state_json[LAST_STATE_JSON_LEN - 1] = '\0';
    s_last_reason_json[0] = '\0';

    // Try MQTT first
    esp_err_t mqtt_ret = ESP_FAIL;
    if (s_mqtt_state_cb != NULL) {
      mqtt_ret = json_tx_submit(true, topic, json_str);
    }

    // Log to serial if enabled or if MQTT failed
    if (s_config.log_to_serial || mqtt_ret != ESP_OK) {
      const char *status = (mqtt_ret == ESP_OK) ? "SENT" : "LOCAL";
      ESP_LOGI(TAG, "[MQTT:%s->%s] %s", status, topic, json_str);
    }

    cJSON_free(json_str);
  }

  cJSON_Delete(root);
}

void json_logger_reason_code(uint8_t reason_code, const char *l1,
                             const char *l2) {
  if (!s_initialized) {
    return;
  }

  // Build JSON
  cJSON *root = cJSON_CreateObject();
  if (root == NULL) {
    ESP_LOGE(TAG, "Failed to create JSON object");
    return;
  }

  // Format reason code as string (per spec)
  char reason_str[4];
  snprintf(reason_str, sizeof(reason_str), "%d", reason_code);

  cJSON_AddStringToObject(root, "type", "reason");
  cJSON_AddStringToObject(root, "client_id", s_config.client_id);
  cJSON_AddStringToObject(root, "reason_code", reason_str);
  cJSON_AddNumberToObject(root, "seq", (double)(++s_state_seq));

  // Add catalog text if available
  if (l1 && l1[0] != '\0') {
    cJSON_AddStringToObject(root, "reason_l1", l1);
  }
  if (l2 && l2[0] != '\0') {
    cJSON_AddStringToObject(root, "reason_l2", l2);
  }

  // Generate JSON string
  char *json_str = cJSON_PrintUnformatted(root);
  if (json_str != NULL) {
    char topic[64];
    build_topic(topic, sizeof(topic), "statechange");

    // Cache for reconnect flush
    strncpy(s_last_reason_json, json_str, LAST_REASON_JSON_LEN - 1);
    s_last_reason_json[LAST_REASON_JSON_LEN - 1] = '\0';

    // Try MQTT first
    esp_err_t mqtt_ret = ESP_FAIL;
    if (s_mqtt_state_cb != NULL) {
      mqtt_ret = json_tx_submit(true, topic, json_str);
    }

    // Log to serial if enabled or if MQTT failed
    if (s_config.log_to_serial || mqtt_ret != ESP_OK) {
      const char *status = (mqtt_ret == ESP_OK) ? "SENT" : "LOCAL";
      ESP_LOGI(TAG, "[MQTT:%s->%s] %s", status, topic, json_str);
    }

    cJSON_free(json_str);
  }

  cJSON_Delete(root);
}

void json_logger_livedata(const json_livedata_t *data) {
  if (!s_initialized || data == NULL) {
    return;
  }

  json_machine_state_t state = data->state;
  if (state > JSON_STATE_STP) {
    state = JSON_STATE_ACT;
  }

  // Build JSON
  cJSON *root = cJSON_CreateObject();
  if (root == NULL) {
    ESP_LOGE(TAG, "Failed to create JSON object");
    return;
  }

  // Add fields with 2 decimal place precision
  cJSON_AddNumberToObject(root, "I", round_2dp(data->current_avg));
  if (!data->modbus_ct_enabled) {
    cJSON_AddNumberToObject(root, "ir", round_2dp(data->phase_current_r));
    cJSON_AddNumberToObject(root, "iy", round_2dp(data->phase_current_y));
    cJSON_AddNumberToObject(root, "ib", round_2dp(data->phase_current_b));
  }
  cJSON_AddNumberToObject(root, "kw", round_2dp(data->power_kw));
  cJSON_AddNumberToObject(root, "pf", round_2dp(data->power_factor));
  if (data->changeover_final) {
    cJSON_AddBoolToObject(root, "changeover_final", true);
  }

  uint32_t part_count = 0;
  if (data->proxy_production_enabled) {
    part_count = data->proxy_good_count;
  } else if (data->encoder_enabled && data->encoder_scrap_mode) {
    part_count = data->encoder_good_count;
  } else if (data->ct_sensor_enabled) {
    part_count = data->ct_part_count;
  } else if (data->duration_count_enabled) {
    part_count = data->duration_finished_count;
  } else if (data->signal_count_enabled[0] || data->signal_count_enabled[1]) {
    if (data->signal_count_enabled[0]) {
      part_count += data->signal_finished_count[0];
    }
    if (data->signal_count_enabled[1]) {
      part_count += data->signal_finished_count[1];
    }
  }
  cJSON_AddNumberToObject(root, "part_count", part_count);
  cJSON_AddNumberToObject(root, "count_total", (double)data->count_total);

  if (data->proxy_production_enabled) {
    if (data->proxy_changeover_final) {
      cJSON_AddBoolToObject(root, "proxy_changeover_final", true);
    }
    cJSON_AddBoolToObject(root, "proxy_counting_enabled", true);
    cJSON_AddStringToObject(root, "proxy_count_mode",
                            data->proxy_meter_mode ? "meter_count"
                                                   : "parts_count");
    cJSON_AddNumberToObject(root, "proxy_a_count", data->count_total1);
    cJSON_AddNumberToObject(root, "proxy_b_count", data->count_total2);
    cJSON_AddNumberToObject(root, "proxy_a_good_count",
                            data->proxy_good_count_a);
    cJSON_AddNumberToObject(root, "proxy_b_good_count",
                            data->proxy_good_count_b);
    cJSON_AddNumberToObject(root, "proxy_good_product_count",
                            data->proxy_good_count);
    cJSON_AddNumberToObject(root, "proxy_scrap_count",
                            data->proxy_scrap_count);
    cJSON_AddNumberToObject(root, "proxy_a_scrap_count",
                            data->proxy_scrap_count_a);
    cJSON_AddNumberToObject(root, "proxy_b_scrap_count",
                            data->proxy_scrap_count_b);
    // Latest complete interval only; omit until valid, never invent a zero.
    if (data->proxy_meter_mode && !data->proxy_changeover_final &&
        !data->changeover_final && data->proxy_minute_ready) {
      cJSON_AddNumberToObject(root, "proxy_count_last_60s", data->proxy_count_last_60s);
    }
    if (data->proxy_meter_mode) {
      cJSON_AddNumberToObject(root, "proxy_total_length_m",
                              round_2dp(data->proxy_total_length_m));
      cJSON_AddNumberToObject(root, "proxy_meter_per_count",
                              round_2dp(data->proxy_meter_per_count));
      cJSON_AddNumberToObject(root, "proxy_cycle_time_s",
                              round_2dp(data->proxy_cycle_time_s));
      cJSON_AddNumberToObject(root, "proxy_last_pulse_s",
                              round_2dp(data->proxy_last_pulse_s));
      cJSON_AddNumberToObject(root, "proxy_a_total_length_m",
                              round_2dp(data->proxy_total_length_a_m));
      cJSON_AddNumberToObject(root, "proxy_b_total_length_m",
                              round_2dp(data->proxy_total_length_b_m));
      cJSON_AddNumberToObject(root, "proxy_a_last_pulse_s",
                              round_2dp(data->proxy_last_pulse_a_s));
      cJSON_AddNumberToObject(root, "proxy_b_last_pulse_s",
                              round_2dp(data->proxy_last_pulse_b_s));
    }
  }
  if (data->encoder_enabled) {
    if (data->encoder_changeover_final) {
      cJSON_AddBoolToObject(root, "encoder_changeover_final", true);
    }
    cJSON_AddNumberToObject(root, "encoder_rpm", round_2dp(data->encoder_rpm));
    cJSON_AddNumberToObject(root, "encoder_mpm", round_2dp(data->encoder_mpm));
    cJSON_AddNumberToObject(root, "encoder_length_m",
                            round_2dp(data->encoder_length_m));
    cJSON_AddNumberToObject(root, "encoder_diameter_mm",
                            round_2dp(data->encoder_diameter_mm));
    cJSON_AddNumberToObject(root, "encoder_count", data->encoder_count);
    cJSON_AddNumberToObject(root, "encoder_good_count",
                            data->encoder_good_count);
    cJSON_AddNumberToObject(root, "encoder_scrap_count",
                            data->encoder_scrap_count);
    if (data->encoder_scrap_mode) {
      cJSON_AddStringToObject(root, "encoder_mode", "scrap");
      cJSON_AddNumberToObject(root, "encoder_target_length_m",
                              round_2dp(data->encoder_target_length_m));
      cJSON_AddNumberToObject(root, "encoder_current_pipe_length_m",
                              round_2dp(data->encoder_current_pipe_length_m));
    } else {
      cJSON_AddStringToObject(root, "encoder_mode", "normal");
    }
  }
  if (data->ct_sensor_enabled) {
    cJSON_AddNumberToObject(root, "ct_current_value",
                            round_2dp(data->ct_current_value));
    cJSON_AddNumberToObject(root, "ct_threshold",
                            round_2dp(data->ct_threshold));
    cJSON_AddNumberToObject(root, "ct_stabilization_ms",
                            data->ct_stabilization_ms);
    cJSON_AddNumberToObject(root, "ct_part_count", data->ct_part_count);
    cJSON_AddBoolToObject(root, "ct_sensor_enabled", true);
  }
  if (data->duration_count_enabled) {
    if (data->duration_changeover_final) {
      cJSON_AddBoolToObject(root, "cycle_time_changeover_final", true);
    }
    cJSON_AddBoolToObject(root, "cycle_time_count_enabled", true);
    cJSON_AddBoolToObject(root, "cycle_time_selected_valid",
                          data->duration_selected_valid);
    cJSON_AddNumberToObject(root, "cycle_time_threshold_a",
                            round_2dp(data->duration_threshold_a));
    if (data->duration_selected_valid) {
      cJSON_AddNumberToObject(root, "cycle_time_part_number",
                              data->duration_part_number);
      cJSON_AddNumberToObject(root, "part_number",
                              data->duration_part_number);
      cJSON_AddNumberToObject(root, "cycle_time_s",
                              data->duration_cycle_time_s);
      cJSON_AddNumberToObject(root, "cycle_time_operations_per_part",
                              data->duration_operations_per_part);
    }
  }
  if (data->signal_count_enabled[0] || data->signal_count_enabled[1]) {
    if (data->signal_count_changeover_final) {
      cJSON_AddBoolToObject(root, "signal_count_changeover_final", true);
    }
    const uint8_t active_signal = data->signal_count_enabled[0] ? 0 : 1;
    cJSON_AddNumberToObject(root, "signal_part_number",
                            data->signal_part_number[active_signal]);
    cJSON_AddBoolToObject(root, "cycle_time_signal_count_enabled", true);
    cJSON_AddNumberToObject(root, "cycle_time_finished_count",
                            data->signal_finished_count[active_signal]);
    cJSON_AddNumberToObject(root, "cycle_time_operation_count",
                            data->signal_operation_count[active_signal]);
    cJSON_AddNumberToObject(root, "signal_count_rearm_s",
                            data->signal_count_rearm_ms / 1000.0);
    cJSON_AddNumberToObject(root, "cycle_time_s",
                            data->signal_cycle_time_s[active_signal]);
  }
  if (data->speed_enabled) {
    cJSON_AddBoolToObject(root, "speed_enabled", true);
    cJSON_AddNumberToObject(root, "speed_current_length_m",
                            round_2dp(data->speed_current_length_m));
    cJSON_AddNumberToObject(root, "speed_total_length_m",
                            round_2dp(data->speed_total_length_m));
    cJSON_AddNumberToObject(root, "speed_mps",
                            round_2dp(data->speed_mps));
    cJSON_AddNumberToObject(root, "speed_threshold_a",
                            round_2dp(data->speed_threshold_a));
  }
  cJSON_AddNumberToObject(root, "today_kwh", round_2dp(data->today_kwh));
  cJSON_AddNumberToObject(root, "total_kwh", round_2dp(data->total_kwh));
  if (data->modbus_ct_enabled) {
    cJSON_AddBoolToObject(root, "modbus_ct_enabled", true);
    cJSON_AddStringToObject(root, "modbus_ct_network",
                            data->modbus_ct_network);
    if (data->modbus_ct_data_valid) {
      cJSON_AddNumberToObject(root, "kva", round_2dp(data->modbus_ct_kva));
      cJSON_AddNumberToObject(root, "kvah_today", round_2dp(data->today_kwh));
      cJSON_AddNumberToObject(root, "kvah_total", round_2dp(data->total_kwh));
      cJSON_AddNumberToObject(root, "vrn", round_2dp(data->modbus_ct_vrn));
      cJSON_AddNumberToObject(root, "vyn", round_2dp(data->modbus_ct_vyn));
      cJSON_AddNumberToObject(root, "vbn", round_2dp(data->modbus_ct_vbn));
      // Phase-neutral aliases expected by the existing DB/UI mapping.
      cJSON_AddNumberToObject(root, "vr", round_2dp(data->modbus_ct_vrn));
      cJSON_AddNumberToObject(root, "vy", round_2dp(data->modbus_ct_vyn));
      cJSON_AddNumberToObject(root, "vb", round_2dp(data->modbus_ct_vbn));
      cJSON_AddNumberToObject(root, "vry", round_2dp(data->modbus_ct_vry));
      cJSON_AddNumberToObject(root, "vyb", round_2dp(data->modbus_ct_vyb));
      cJSON_AddNumberToObject(root, "vbr", round_2dp(data->modbus_ct_vbr));
      cJSON_AddNumberToObject(root, "ir", round_2dp(data->modbus_ct_ir));
      cJSON_AddNumberToObject(root, "iy", round_2dp(data->modbus_ct_iy));
      cJSON_AddNumberToObject(root, "ib", round_2dp(data->modbus_ct_ib));
      cJSON_AddNumberToObject(root, "i_avg", round_2dp(data->modbus_ct_i_avg));
    } else {
      cJSON_AddNullToObject(root, "kva");
      cJSON_AddNullToObject(root, "kvah_today");
      cJSON_AddNullToObject(root, "kvah_total");
      cJSON_AddNullToObject(root, "ir");
      cJSON_AddNullToObject(root, "iy");
      cJSON_AddNullToObject(root, "ib");
    }
  }
  cJSON_AddBoolToObject(root, "energy_reset_event", data->energy_reset_event);
  if (data->energy_reset_event && data->energy_reset_reason[0] != '\0') {
    cJSON_AddStringToObject(root, "energy_reset_reason",
                            data->energy_reset_reason);
  }
  cJSON_AddStringToObject(root, "state", JSON_STATE_NAMES[state]);
  if (data->vib_enabled) {
    if (data->vib_data_valid) {
      cJSON_AddNumberToObject(root, "vib_vel_avg", round_2dp(data->vib_vel_avg));
      cJSON_AddNumberToObject(root, "vib_vel_peak", round_2dp(data->vib_vel_peak));
      cJSON_AddNumberToObject(root, "vib_acc_avg", round_2dp(data->vib_acc_avg));
      cJSON_AddNumberToObject(root, "vib_acc_peak", round_2dp(data->vib_acc_peak));
      cJSON_AddNumberToObject(root, "vib_temp",
                              (double)((int)(data->vib_temp * 10 + 0.5f)) / 10.0);
    } else {
      cJSON_AddNullToObject(root, "vib_vel_avg");
      cJSON_AddNullToObject(root, "vib_vel_peak");
      cJSON_AddNullToObject(root, "vib_acc_avg");
      cJSON_AddNullToObject(root, "vib_acc_peak");
      cJSON_AddNullToObject(root, "vib_temp");
    }
  }

  // Generate JSON string
  char *json_str = cJSON_PrintUnformatted(root);
  if (json_str != NULL) {
    char topic[64];
    build_topic(topic, sizeof(topic), "livedata");

    // Try MQTT first
    esp_err_t mqtt_ret = ESP_FAIL;
    if (s_mqtt_livedata_cb != NULL) {
      mqtt_ret = json_tx_submit(false, topic, json_str);
    }

    // Log to serial if enabled or if MQTT failed
    if (s_config.log_to_serial || mqtt_ret != ESP_OK) {
      const char *status = (mqtt_ret == ESP_OK) ? "SENT" : "LOCAL";
      ESP_LOGI(TAG, "[MQTT:%s->%s] %s", status, topic, json_str);
    }

    cJSON_free(json_str);
  }

  cJSON_Delete(root);
}

bool json_logger_periodic_check(const json_livedata_t *data) {
  if (!s_initialized || data == NULL) {
    return false;
  }

  uint32_t now_ms = get_time_ms();
  uint32_t elapsed = now_ms - s_last_livedata_ms;

  if (elapsed >= s_config.livedata_interval_ms) {
    s_last_livedata_ms = now_ms;
    json_logger_livedata(data);
    return true;
  }

  return false;
}

void json_logger_livedata_immediate(const json_livedata_t *data) {
  if (!s_initialized || data == NULL) {
    return;
  }

  // Send livedata
  json_logger_livedata(data);

  // Reset periodic timer to prevent double-logging
  s_last_livedata_ms = get_time_ms();
}

// ============================================================================
// Health Diagnostics JSON (published every 30s)
// ============================================================================

void json_logger_health(const json_health_t *data) {
  if (!s_initialized || data == NULL) {
    return;
  }

  cJSON *root = cJSON_CreateObject();
  if (root == NULL) {
    ESP_LOGE(TAG, "Failed to create health JSON object");
    return;
  }

  // Top-level fields
  cJSON_AddStringToObject(root, "type", "health");
  cJSON_AddStringToObject(root, "client_id", s_config.client_id);
  /* Device identity metadata allows the dashboard to register a device from
   * its first MQTT health message without a manual database entry. */
  cJSON_AddStringToObject(root, "product", "Datameter");
  cJSON_AddStringToObject(root, "hw", "ESP32-S3");

  if (data->firmware_version[0] != '\0') {
    cJSON_AddStringToObject(root, "fw", data->firmware_version);
  }
  if (data->mac_id[0] != '\0') {
    cJSON_AddStringToObject(root, "mac_id", data->mac_id);
  }

  // Fault trigger: CT open, hardware errors, or peripheral init failures (#ISSUE-53/#ISSUE-54)
  bool is_fault = (data->ct_open_mask != 0) ||
                  (data->i2c_err_count > 0) ||
                  (data->ads_err_count > 0) ||
                  (!data->relay_gpio_ok) ||
                  (!data->lcd_ok) ||
                  (!data->keypad_ok);
  cJSON_AddBoolToObject(root, "is_fault_trigger", is_fault);

  // --- Uplink ---
  cJSON *uplink = cJSON_CreateObject();
  if (uplink) {
    const char *mode_str = (data->uplink_mode == 1) ? "LTE" : "WiFi";
    cJSON_AddStringToObject(uplink, "mode", mode_str);

    if (data->uplink_mode == 1) {
      cJSON *lte = cJSON_CreateObject();
      if (lte) {
        cJSON_AddNumberToObject(lte, "rssi_csq", data->lte_rssi_csq);
        cJSON_AddStringToObject(lte, "status",
                                data->lte_connected ? "CONNECTED" : "DISCONNECTED");
        cJSON_AddStringToObject(lte, "sim_phone",
                                data->lte_phone[0] ? data->lte_phone : "");
        cJSON_AddStringToObject(lte, "recovery_state",
                                data->lte_recovery_state);
        cJSON_AddStringToObject(lte, "last_error", data->lte_last_error);
        cJSON_AddNumberToObject(lte, "consecutive_publish_failures",
                                data->lte_consecutive_publish_failures);
        cJSON_AddNumberToObject(lte, "connect_attempts",
                                data->lte_connect_attempts);
        cJSON_AddNumberToObject(lte, "mqtt_reconnects",
                                data->lte_mqtt_reconnects);
        cJSON_AddNumberToObject(lte, "modem_restarts",
                                data->lte_modem_restarts);
        cJSON_AddStringToObject(lte, "last_recovery_action",
                                data->lte_last_recovery_action);
        cJSON_AddItemToObject(uplink, "lte", lte);
      }
    } else {
      cJSON *wifi = cJSON_CreateObject();
      if (wifi) {
        cJSON_AddStringToObject(wifi, "ssid",
                                data->wifi_ssid[0] ? data->wifi_ssid : "");
        cJSON_AddBoolToObject(wifi, "pass_set", data->wifi_pass_set);
        cJSON_AddNumberToObject(wifi, "rssi", data->wifi_rssi);
        cJSON_AddStringToObject(wifi, "status",
                                data->wifi_connected ? "OK" : "DISCONNECTED");
        cJSON_AddItemToObject(uplink, "wifi", wifi);
      }
    }

    cJSON_AddItemToObject(root, "uplink", uplink);
  }

  // --- Sensors ---
  cJSON *sensors = cJSON_CreateObject();
  if (sensors) {
    // CT sensor (available)
    cJSON *ct = cJSON_CreateObject();
    if (ct) {
      cJSON_AddStringToObject(ct, "status",
                              (data->ct_open_mask == 0) ? "OK" : "OPEN");
      cJSON_AddNumberToObject(ct, "raw", round_2dp(data->ct_phase_a_vrms));
      cJSON_AddItemToObject(sensors, "ct", ct);
    }

    // Relay actuator — reports GPIO init status (#ISSUE-53)
    cJSON *relay = cJSON_CreateObject();
    if (relay) {
      cJSON_AddStringToObject(relay, "status",
                              data->relay_gpio_ok ? "OK" : "GPIO_FAULT");
      cJSON_AddItemToObject(sensors, "relay", relay);
    }

    // LCD display — reports init status (#ISSUE-54)
    cJSON *lcd = cJSON_CreateObject();
    if (lcd) {
      cJSON_AddStringToObject(lcd, "status", data->lcd_ok ? "OK" : "INIT_FAULT");
      cJSON_AddItemToObject(sensors, "lcd", lcd);
    }

    // Keypad — reports init status (#ISSUE-54)
    cJSON *keypad = cJSON_CreateObject();
    if (keypad) {
      cJSON_AddStringToObject(keypad, "status", data->keypad_ok ? "OK" : "INIT_FAULT");
      cJSON_AddItemToObject(sensors, "keypad", keypad);
    }

    // Vibration sensor — live data, replaces stub
    {
      cJSON *vib = cJSON_CreateObject();
      if (vib) {
        cJSON_AddStringToObject(vib, "model", data->vib_model[0] ? data->vib_model : "UNKNOWN");
        const char *vib_status;
        if (!data->vib_init_ok) {
          vib_status = "INIT_FAULT";
        } else if (data->vib_comm_error) {
          vib_status = "COMM_ERROR";
        } else if (!data->vib_data_valid) {
          vib_status = "NO_DATA";
        } else {
          vib_status = "OK";
        }
        cJSON_AddStringToObject(vib, "status", vib_status);
        if (data->vib_data_valid) {
          cJSON_AddNumberToObject(vib, "vel_rms_mm_s", round_2dp(data->vib_vel_rms));
          cJSON_AddNumberToObject(vib, "temp_c",
                                  (double)((int)(data->vib_temp * 10 + 0.5f)) / 10.0);
        }
        cJSON_AddNumberToObject(vib, "err_count", data->vib_err_count);
        cJSON_AddItemToObject(sensors, "vib", vib);
      }
    }

    cJSON_AddItemToObject(root, "sensors", sensors);
  }

  // --- Timing ---
  cJSON *timing = cJSON_CreateObject();
  if (timing) {
    cJSON_AddNumberToObject(timing, "uptime_sec", data->uptime_sec);
    cJSON_AddNumberToObject(timing, "boot_count", data->boot_count);
    cJSON_AddStringToObject(timing, "boot_reason", data->boot_reason);
    cJSON_AddItemToObject(root, "timing", timing);
  }

  // --- Memory ---
  cJSON *mem = cJSON_CreateObject();
  if (mem) {
    cJSON_AddNumberToObject(mem, "free_heap", data->free_heap);
    cJSON_AddNumberToObject(mem, "min_heap", data->min_heap);
    cJSON_AddItemToObject(root, "mem", mem);
  }

  // --- Errors array ---
  cJSON *errors = cJSON_CreateArray();
  if (errors) {
    if (data->ct_open_mask & 0x01)
      cJSON_AddItemToArray(errors, cJSON_CreateString("CT_PHASE_A_OPEN"));
    if (data->ct_open_mask & 0x02)
      cJSON_AddItemToArray(errors, cJSON_CreateString("CT_PHASE_B_OPEN"));
    if (data->ct_open_mask & 0x04)
      cJSON_AddItemToArray(errors, cJSON_CreateString("CT_PHASE_C_OPEN"));
    if (data->i2c_err_count > 0)
      cJSON_AddItemToArray(errors, cJSON_CreateString("I2C_ERRORS"));
    if (data->ads_err_count > 0)
      cJSON_AddItemToArray(errors, cJSON_CreateString("ADS_ERRORS"));
    if (!data->relay_gpio_ok)  /* #ISSUE-53: relay pin floating — GPIO config failed at boot */
      cJSON_AddItemToArray(errors, cJSON_CreateString("RELAY_GPIO_FAULT"));
    if (!data->lcd_ok)         /* #ISSUE-54: LCD init failed — running headlessly */
      cJSON_AddItemToArray(errors, cJSON_CreateString("LCD_INIT_FAULT"));
    if (!data->keypad_ok)      /* #ISSUE-54: keypad init failed — keypad input disabled */
      cJSON_AddItemToArray(errors, cJSON_CreateString("KEYPAD_INIT_FAULT"));
    if (!data->vib_init_ok)
      cJSON_AddItemToArray(errors, cJSON_CreateString("VIB_INIT_FAULT"));
    else if (data->vib_comm_error)
      cJSON_AddItemToArray(errors, cJSON_CreateString("VIB_COMM_ERROR"));
    if (data->modbus_ct_enabled && !data->modbus_ct_data_valid)
      cJSON_AddItemToArray(errors, cJSON_CreateString("MODBUS_CT_COMM_ERROR"));
    cJSON_AddItemToObject(root, "errors", errors);
  }

  // --- Params ---
  cJSON *params = cJSON_CreateObject();
  if (params) {
    cJSON *ct_gain_arr = cJSON_CreateArray();
    if (ct_gain_arr) {
      cJSON_AddItemToArray(ct_gain_arr, cJSON_CreateNumber(round_2dp(data->ct_gain[0])));
      cJSON_AddItemToArray(ct_gain_arr, cJSON_CreateNumber(round_2dp(data->ct_gain[1])));
      cJSON_AddItemToArray(ct_gain_arr, cJSON_CreateNumber(round_2dp(data->ct_gain[2])));
      cJSON_AddItemToObject(params, "ct_gain", ct_gain_arr);
    }
    cJSON_AddNumberToObject(params, "machine_setup_ms", data->machine_setup_time_ms);
    cJSON_AddNumberToObject(params, "kw_threshold", round_2dp(data->kw_threshold));
    cJSON_AddNumberToObject(params, "kw_hysteresis", round_2dp(data->kw_hysteresis));
    cJSON_AddBoolToObject(params, "modbus_ct_enabled", data->modbus_ct_enabled);
    cJSON_AddBoolToObject(params, "modbus_ct_ok", data->modbus_ct_data_valid);
    cJSON_AddNumberToObject(params, "modbus_ct_err_count",
                            data->modbus_ct_err_count);
    cJSON_AddItemToObject(root, "params", params);
  }

  // --- State Detection Configuration ---
  // New configuration-only block. Existing health fields and hierarchy above
  // remain unchanged.
  nvs_device_config_t cfg_snap;
  const nvs_device_config_t *cfg =
      nvs_config_snapshot(&cfg_snap) ? &cfg_snap : NULL;
  if (cfg) {
    cJSON *state_cfg = cJSON_CreateObject();
    if (state_cfg) {
      const char *selected_source = "CT SENSOR";
      if (cfg->state_source == NVS_STATE_SRC_SIGNAL) {
        selected_source = "24V SIGNAL";
      } else if (cfg->state_source == NVS_STATE_SRC_MODBUS_CT &&
                 cfg->modbus_ct_enabled) {
        selected_source = "ENERGYMETER";
      }
      cJSON_AddStringToObject(state_cfg, "selected_source", selected_source);

      cJSON *ct_cfg = cfg->state_source == NVS_STATE_SRC_CT
                          ? cJSON_CreateObject() : NULL;
      if (ct_cfg) {
        cJSON_AddNumberToObject(ct_cfg, "threshold_kw",
                                round_2dp(cfg->active_threshold));
        cJSON_AddNumberToObject(ct_cfg, "hysteresis_factor",
                                round_2dp(cfg->hysteresis_factor));
        cJSON_AddNumberToObject(ct_cfg, "run_to_idle_s",
                                cfg->state_idle_stabilization_ms / 1000.0);
        cJSON_AddItemToObject(state_cfg, "ct_sensor_settings", ct_cfg);
      }

      cJSON *meter_cfg =
          (cfg->state_source == NVS_STATE_SRC_MODBUS_CT &&
           cfg->modbus_ct_enabled)
              ? cJSON_CreateObject() : NULL;
      if (meter_cfg) {
        const char *network = "3PH-4WIRE";
        if (cfg->modbus_ct_network == NVS_MODBUS_CT_NET_3P3W) {
          network = "3PH-3WIRE";
        } else if (cfg->modbus_ct_network == NVS_MODBUS_CT_NET_1P2W) {
          network = "1PH-2WIRE";
        }
        cJSON_AddBoolToObject(meter_cfg, "configured_enabled",
                              cfg->modbus_ct_enabled);
        cJSON_AddStringToObject(
            meter_cfg, "model",
            cfg->modbus_ct_model == NVS_ENERGY_METER_AVH_14_M1
                ? "AVH-14-M1"
                : "AVF-133-M1");
        cJSON_AddStringToObject(meter_cfg, "wiring_mode", network);
        cJSON_AddNumberToObject(meter_cfg, "threshold_kw",
                                round_2dp(cfg->modbus_ct_threshold_kva));
        cJSON_AddNumberToObject(meter_cfg, "hysteresis_factor",
                                round_2dp(cfg->modbus_ct_hysteresis_factor));
        cJSON_AddNumberToObject(
            meter_cfg, "run_to_idle_s",
            cfg->modbus_ct_idle_stabilization_ms / 1000.0);
        cJSON_AddItemToObject(state_cfg, "energymeter_settings", meter_cfg);
      }

      cJSON *signal_cfg = cfg->state_source == NVS_STATE_SRC_SIGNAL
                              ? cJSON_CreateObject() : NULL;
      if (signal_cfg) {
        cJSON_AddStringToObject(
            signal_cfg, "input_logic",
            cfg->signal_logic == NVS_SIGNAL_LOGIC_NC ? "NC" : "NO");
        cJSON_AddNumberToObject(signal_cfg, "run_to_idle_s",
                                cfg->state_idle_stabilization_ms / 1000.0);
        cJSON_AddItemToObject(state_cfg, "signal_24v_settings", signal_cfg);
      }
      cJSON_AddItemToObject(root, "state_detection", state_cfg);
    }

    // --- Production Configuration ---
    cJSON *production_cfg = cJSON_CreateObject();
    if (production_cfg) {
      cJSON *proxy_cfg = cfg->proxy_counting_enabled
                             ? cJSON_CreateObject() : NULL;
      if (proxy_cfg) {
        cJSON_AddBoolToObject(proxy_cfg, "configured_enabled",
                              cfg->proxy_counting_enabled);
        cJSON_AddNumberToObject(proxy_cfg, "input_mask",
                                cfg->proxy_enabled_mask);
        cJSON_AddStringToObject(
            proxy_cfg, "counting_mode",
            cfg->proxy_count_mode == NVS_PROXY_COUNT_MODE_METER
                ? "METER"
                : "PARTS");
        cJSON_AddNumberToObject(proxy_cfg, "debounce_ms",
                                cfg->proxy_debounce_ms);
        cJSON_AddNumberToObject(proxy_cfg, "configured_meter_per_count",
                                round_2dp(cfg->proxy_meter_per_count));
        cJSON_AddBoolToObject(proxy_cfg, "scrap_classification_enabled",
                              cfg->proxy_scrap_enabled);
        cJSON_AddNumberToObject(proxy_cfg, "scrap_cycle_time_s",
                                round_2dp(cfg->proxy_scrap_cycle_time_s));
        cJSON_AddItemToObject(production_cfg, "proxy_settings", proxy_cfg);
      }

      cJSON *encoder_cfg = cfg->encoder_enabled
                               ? cJSON_CreateObject() : NULL;
      if (encoder_cfg) {
        cJSON_AddBoolToObject(encoder_cfg, "configured_enabled",
                              cfg->encoder_enabled);
        cJSON_AddStringToObject(
            encoder_cfg, "operating_mode",
            cfg->encoder_mode == NVS_ENCODER_MODE_SCRAP ? "SCRAP" : "NORMAL");
        cJSON_AddNumberToObject(encoder_cfg, "pulses_per_revolution",
                                round_2dp(cfg->encoder_ppr));
        cJSON_AddNumberToObject(encoder_cfg, "mpm_calibration_factor",
                                round_2dp(cfg->encoder_mpm_factor));
        cJSON_AddStringToObject(
            encoder_cfg, "positive_direction",
            cfg->encoder_direction == NVS_ENCODER_DIR_ANTICLOCKWISE
                ? "ANTICLOCKWISE"
                : "CLOCKWISE");
        cJSON_AddNumberToObject(encoder_cfg, "roller_diameter_mm",
                                round_2dp(cfg->encoder_diameter_mm));
        cJSON_AddBoolToObject(encoder_cfg, "scrap_classification_enabled",
                              cfg->encoder_scrap_enabled);
        cJSON_AddNumberToObject(encoder_cfg, "configured_target_length_m",
                                round_2dp(cfg->encoder_scrap_length_m));
        cJSON_AddItemToObject(production_cfg, "encoder_settings", encoder_cfg);
      }

      cJSON *ct_count_cfg = cfg->automize_ct_part_enabled
                                ? cJSON_CreateObject() : NULL;
      if (ct_count_cfg) {
        cJSON_AddBoolToObject(ct_count_cfg, "configured_enabled",
                              cfg->automize_ct_part_enabled);
        cJSON_AddNumberToObject(ct_count_cfg, "operation_threshold_a",
                                round_2dp(cfg->automize_ct_part_threshold_a));
        cJSON_AddNumberToObject(
            ct_count_cfg, "stabilization_s",
            cfg->automize_ct_part_stabilization_ms / 1000.0);
        cJSON_AddItemToObject(production_cfg, "ct_sensor_settings",
                              ct_count_cfg);
      }

      nvs_duration_store_t duration_store;
      bool duration_loaded =
          nvs_config_load_duration(&duration_store) == ESP_OK;
      cJSON *cycle_cfg =
          (duration_loaded && duration_store.count_enabled)
              ? cJSON_CreateObject() : NULL;
      if (cycle_cfg) {
        cJSON_AddStringToObject(
            cycle_cfg, "operation_source",
            cfg->cycle_count_source == NVS_CYCLE_COUNT_SOURCE_SIGNAL
                ? "24V SIGNAL"
                : "CT SENSOR");
        cJSON_AddNumberToObject(cycle_cfg, "operation_threshold_a",
                                round_2dp(cfg->cycle_time_count_threshold_a));
        cJSON_AddItemToObject(production_cfg, "cycle_time_settings", cycle_cfg);
      }

      cJSON *signal_count_cfg =
          (duration_loaded && duration_store.signal_enabled_mask != 0)
              ? cJSON_CreateObject() : NULL;
      if (signal_count_cfg) {
        cJSON_AddStringToObject(
            signal_count_cfg, "signal_a_logic",
            cfg->cycle_signal_logic_a == NVS_SIGNAL_LOGIC_NC ? "NC" : "NO");
        cJSON_AddStringToObject(
            signal_count_cfg, "signal_b_logic",
            cfg->cycle_signal_logic_b == NVS_SIGNAL_LOGIC_NC ? "NC" : "NO");
        cJSON_AddNumberToObject(signal_count_cfg, "rearm_s",
                                cfg->cycle_signal_rearm_ms / 1000.0);
        cJSON_AddItemToObject(production_cfg, "signal_24v_settings",
                              signal_count_cfg);
      }

      cJSON *speed_cfg = cfg->speed_enabled
                             ? cJSON_CreateObject() : NULL;
      if (speed_cfg) {
        cJSON_AddBoolToObject(speed_cfg, "configured_enabled",
                              cfg->speed_enabled);
        cJSON_AddNumberToObject(speed_cfg, "start_threshold_a",
                                round_2dp(cfg->speed_threshold_a));
        cJSON_AddNumberToObject(speed_cfg, "configured_speed_mps",
                                round_2dp(cfg->speed_mps));
        cJSON_AddItemToObject(production_cfg, "speed_settings", speed_cfg);
      }

      const char *scrap_source = "NONE";
      if (cfg->scrap_source == NVS_SCRAP_SOURCE_PROXY &&
          cfg->proxy_counting_enabled && cfg->proxy_scrap_enabled) {
        scrap_source = "PROXY";
      } else if (cfg->scrap_source == NVS_SCRAP_SOURCE_ENCODER &&
                 cfg->encoder_enabled && cfg->encoder_scrap_enabled) {
        scrap_source = "ENCODER";
      } else if (cfg->scrap_source == NVS_SCRAP_SOURCE_DURATION) {
        scrap_source = "CYCLE TIME";
      }
      cJSON_AddStringToObject(production_cfg, "selected_scrap_source",
                              scrap_source);
      cJSON_AddItemToObject(root, "production", production_cfg);
    }
  }

  // --- Publish ---
  char *json_str = cJSON_PrintUnformatted(root);
  if (json_str != NULL) {
    char topic[64];
    build_topic(topic, sizeof(topic), "health");

    // Use state callback (QoS 1)
    esp_err_t mqtt_ret = ESP_FAIL;
    if (s_mqtt_state_cb != NULL) {
      mqtt_ret = json_tx_submit(true, topic, json_str);
    }

    if (s_config.log_to_serial || mqtt_ret != ESP_OK) {
      const char *status = (mqtt_ret == ESP_OK) ? "SENT" : "LOCAL";
      ESP_LOGI(TAG, "[MQTT:%s->%s] %s", status, topic, json_str);
    }

    cJSON_free(json_str);
  }

  cJSON_Delete(root);
}

bool json_logger_health_periodic_check(const json_health_t *data) {
  if (!s_initialized || data == NULL) {
    return false;
  }

  uint32_t now_ms = get_time_ms();
  uint32_t elapsed = now_ms - s_last_health_ms;

  if (elapsed >= HEALTH_INTERVAL_MS) {
    s_last_health_ms = now_ms;
    json_logger_health(data);
    return true;
  }

  return false;
}
