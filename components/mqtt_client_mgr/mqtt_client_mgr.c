/**
 * @file mqtt_client_mgr.c
 * @brief MQTT Client Manager implementation for CT+HMI System
 *
 * Uses ESP-IDF MQTT client with event-driven connection management.
 * Auto-reconnect is handled internally by ESP-MQTT.
 */

#include "mqtt_client_mgr.h"
#include "json_data_logger.h"
#include "cJSON.h"
#include "log_config.h"
#undef LOG_LOCAL_LEVEL
#define LOG_LOCAL_LEVEL LOG_LEVEL_MQTT
#include "esp_log.h"
#include "esp_crt_bundle.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "mqtt_client.h"
#include "nvs_config.h"
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

static const char *TAG = "mqtt_client_mgr";

// MQTT Configuration Constants
// MQTT_KEEPALIVE_SEC is now loaded from NVS
#define MQTT_RECONNECT_TIMEOUT_MS 5000 // Reconnect timeout in milliseconds (increased from 5000 to prevent frequent disconnections on slow networks)
#define MQTT_BUFFER_SIZE 2048 // MQTT buffer size (config payload can be ~1.5KB)

// LWT (Last Will and Testament) Configuration
#define MQTT_LWT_QOS 1       // QoS for LWT message
#define MQTT_LWT_RETAIN true // Retain LWT message
#define MQTT_LWT_MSG_OFFLINE "offline"
#define MQTT_BIRTH_MSG_ONLINE "online"

// Module state
static struct {
  mqtt_client_config_t config;
  mqtt_status_t status;
  SemaphoreHandle_t status_mutex; // protects status (cross-core read/write)
  esp_mqtt_client_handle_t client;
  char state_topic[96];      // Full topic: {base_topic}/statechange
  char livedata_topic[96];   // Full topic: {base_topic}/livedata
  char lwt_topic[96];        // Full topic: {base_topic}/lwt (for LWT and birth
                             // messages)
  char config_topic[96];     // Full topic: {base_topic}/config
  char config_ack_topic[96]; // Full topic: {base_topic}/config/ack
  bool initialized;
  bool started;
} s_mqtt = {0};

// Config message callback
static mqtt_config_cb_t s_config_cb = NULL;

// Maximum accepted config payload size (bytes).
// A full reason catalog (20 entries × ~60 bytes each + JSON overhead) fits in ~2500 bytes.
// 4096 gives 60% headroom and prevents heap exhaustion.
#define MQTT_MAX_CONFIG_PAYLOAD 4096

// Fragment reassembly state for config messages that arrive in multiple MQTT_EVENT_DATA events.
// ESP-MQTT fragments any message larger than buffer.size (MQTT_BUFFER_SIZE = 2048).
// Only one config message can be in-flight at a time (QoS 0 subscription).
static struct {
    char *buf;       // Heap-allocated reassembly buffer; NULL when idle
    int   total_len; // Expected total length (from event->total_data_len on first fragment)
    int   received;  // Bytes accumulated so far
} s_frag = {0};

static void trim_ascii(char *s) {
  if (s == NULL || s[0] == '\0') {
    return;
  }

  char *start = s;
  while (*start != '\0' && isspace((unsigned char)*start)) {
    start++;
  }

  char *end = start + strlen(start);
  while (end > start && isspace((unsigned char)*(end - 1))) {
    end--;
  }
  *end = '\0';

  if (start != s) {
    memmove(s, start, strlen(start) + 1);
  }
}

static bool all_digits(const char *s) {
  if (s == NULL || s[0] == '\0') {
    return false;
  }
  for (const char *p = s; *p != '\0'; p++) {
    if (!isdigit((unsigned char)*p)) {
      return false;
    }
  }
  return true;
}

static esp_err_t build_broker_uri(char *out, size_t out_len,
                                  const mqtt_client_config_t *config) {
  if (out == NULL || out_len == 0 || config == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  char broker[sizeof(config->broker_host)];
  strncpy(broker, config->broker_host, sizeof(broker) - 1);
  broker[sizeof(broker) - 1] = '\0';
  trim_ascii(broker);

  if (broker[0] == '\0') {
    ESP_LOGE(TAG, "MQTT broker host is empty");
    return ESP_ERR_INVALID_ARG;
  }

  const char *scheme = "mqtt";
  char *host = broker;
  char *scheme_sep = strstr(broker, "://");
  if (scheme_sep != NULL) {
    *scheme_sep = '\0';
    scheme = broker;
    host = scheme_sep + 3;
  }

  char *path = strchr(host, '/');
  if (path != NULL) {
    *path = '\0';
  }
  trim_ascii(host);

  if (host[0] == '\0') {
    ESP_LOGE(TAG, "MQTT broker URI has no host");
    return ESP_ERR_INVALID_ARG;
  }

  uint16_t port = config->broker_port ? config->broker_port : DEFAULT_MQTT_PORT;

  char *colon = strrchr(host, ':');
  if (colon != NULL && all_digits(colon + 1)) {
    long parsed_port = strtol(colon + 1, NULL, 10);
    if (parsed_port > 0 && parsed_port <= 65535) {
      port = (uint16_t)parsed_port;
      *colon = '\0';
    }
  }

  int written = snprintf(out, out_len, "%s://%s:%u", scheme, host,
                         (unsigned)port);
  if (written < 0 || written >= (int)out_len) {
    ESP_LOGE(TAG, "MQTT broker URI too long");
    return ESP_ERR_INVALID_SIZE;
  }

  return ESP_OK;
}

/**
 * @brief MQTT event handler
 */
static void mqtt_event_handler(void *handler_args, esp_event_base_t base,
                               int32_t event_id, void *event_data) {
  esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;

  switch ((esp_mqtt_event_id_t)event_id) {
  case MQTT_EVENT_CONNECTED:
    /* #ISSUE-46: bounded timeout — portMAX_DELAY in the default event loop task
     * blocks ALL system events (WiFi, IP, MQTT) if status_mutex is contended.
     * Pattern: same as wifi_manager.c:114,170 fixed in #ISSUE-42. */
    if (xSemaphoreTake(s_mqtt.status_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
      s_mqtt.status.connected = true;
      xSemaphoreGive(s_mqtt.status_mutex);
    } else {
      ESP_LOGW(TAG, "status_mutex timeout in CONNECTED handler — status not updated");
    }
    ESP_LOGI(TAG, "Connected to MQTT broker");
    // Publish birth message (online status) with retain
    if (s_mqtt.lwt_topic[0] != '\0') {
      int msg_id = esp_mqtt_client_publish(
          s_mqtt.client, s_mqtt.lwt_topic, MQTT_BIRTH_MSG_ONLINE,
          strlen(MQTT_BIRTH_MSG_ONLINE), 1, 1); // QoS 1, retain=true
      if (msg_id >= 0) {
        ESP_LOGI(TAG, "Published birth message to %s", s_mqtt.lwt_topic);
      } else {
        ESP_LOGW(TAG, "Failed to publish birth message");
      }
    }
    // Flush any statechange/reason JSON missed during the connectivity gap
    json_logger_flush_pending();
    // Subscribe to config topic
    if (s_mqtt.config_topic[0] != '\0') {
      int sub_id =
          esp_mqtt_client_subscribe(s_mqtt.client, s_mqtt.config_topic, 1);
      if (sub_id >= 0) {
        ESP_LOGI(TAG, "Subscribed to config topic: %s", s_mqtt.config_topic);
      } else {
        ESP_LOGE(TAG, "Failed to subscribe to config topic");
      }
    }
    break;

  case MQTT_EVENT_DISCONNECTED:
    /* #ISSUE-46: bounded timeout — same reasoning as MQTT_EVENT_CONNECTED above. */
    if (xSemaphoreTake(s_mqtt.status_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
      s_mqtt.status.connected = false;
      xSemaphoreGive(s_mqtt.status_mutex);
    } else {
      ESP_LOGW(TAG, "status_mutex timeout in DISCONNECTED handler — status not updated");
    }
    // If a fragmented message was being reassembled, discard it — the remaining
    // fragments will never arrive after a disconnect.
    if (s_frag.buf != NULL) {
      ESP_LOGW(TAG, "MQTT disconnected mid-reassembly — discarding %d/%d bytes",
               s_frag.received, s_frag.total_len);
      free(s_frag.buf);
      s_frag.buf = NULL;
      s_frag.total_len = 0;
      s_frag.received = 0;
    }
    ESP_LOGW(TAG, "Disconnected from MQTT broker");
    break;

  case MQTT_EVENT_PUBLISHED:
    // Message successfully published (for QoS 1/2)
    ESP_LOGD(TAG, "Message published, msg_id=%d", event->msg_id);
    break;

  case MQTT_EVENT_ERROR:
    ESP_LOGE(TAG, "MQTT error occurred");
    if (event->error_handle == NULL) {
      break;
    }
    if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
      ESP_LOGE(TAG,
               "MQTT transport error: esp_tls=0x%x tls_stack=0x%x sock_errno=%d",
               (unsigned)event->error_handle->esp_tls_last_esp_err,
               (unsigned)event->error_handle->esp_tls_stack_err,
               event->error_handle->esp_transport_sock_errno);
    } else if (event->error_handle->error_type == MQTT_ERROR_TYPE_CONNECTION_REFUSED) {
      ESP_LOGE(TAG, "MQTT connection refused, return_code=%d",
               event->error_handle->connect_return_code);
    }
    break;

  case MQTT_EVENT_BEFORE_CONNECT:
    ESP_LOGI(TAG, "Connecting to MQTT broker %s:%d...",
             s_mqtt.config.broker_host, s_mqtt.config.broker_port);
    break;

  case MQTT_EVENT_DATA:
    ESP_LOGI(TAG, "MQTT data received: topic_len=%d data_len=%d offset=%d total=%d",
             event->topic_len, event->data_len,
             event->current_data_offset, event->total_data_len);

    // ── First fragment: current_data_offset == 0, topic is populated ──────────
    if (event->current_data_offset == 0) {
      // Discard any leftover state from a previously abandoned message
      if (s_frag.buf != NULL) {
        ESP_LOGW(TAG, "New message arrived while reassembly in progress — discarding previous");
        free(s_frag.buf);
        s_frag.buf = NULL;
        s_frag.total_len = 0;
        s_frag.received = 0;
      }

      // Topic check — only valid on the first fragment
      if (event->topic_len <= 0 || event->data_len <= 0 ||
          event->topic_len != (int)strlen(s_mqtt.config_topic) ||
          strncmp(event->topic, s_mqtt.config_topic, event->topic_len) != 0) {
        break;  // Not a config message — nothing to do
      }

      // total_data_len == 0 means the whole message fits in one fragment;
      // treat it as total_len == data_len in that case.
      int total = (event->total_data_len > 0) ? event->total_data_len : event->data_len;

      if (total > MQTT_MAX_CONFIG_PAYLOAD) {
        ESP_LOGW(TAG, "Config payload too large (%d bytes, limit %d) — dropping",
                 total, MQTT_MAX_CONFIG_PAYLOAD);
        break;
      }

      s_frag.buf = malloc(total + 1);  // +1 for null terminator
      if (s_frag.buf == NULL) {
        ESP_LOGE(TAG, "Failed to allocate reassembly buffer (%d bytes)", total + 1);
        break;
      }
      s_frag.total_len = total;
      s_frag.received = 0;
      ESP_LOGI(TAG, "Config message start: expecting %d bytes total", total);
    }

    // ── All fragments (including first) ───────────────────────────────────────
    // If buf is NULL here, the first-fragment block above bailed (wrong topic,
    // alloc failure, or payload too large) — skip silently.
    if (s_frag.buf == NULL) {
      break;
    }

    // Bounds check before copying
    if (s_frag.received + event->data_len > s_frag.total_len) {
      ESP_LOGE(TAG, "Fragment overflows reassembly buffer (%d + %d > %d) — discarding",
               s_frag.received, event->data_len, s_frag.total_len);
      free(s_frag.buf);
      s_frag.buf = NULL;
      s_frag.total_len = 0;
      s_frag.received = 0;
      break;
    }

    memcpy(s_frag.buf + s_frag.received, event->data, event->data_len);
    s_frag.received += event->data_len;
    ESP_LOGD(TAG, "Fragment appended: progress %d/%d bytes",
             s_frag.received, s_frag.total_len);

    // ── Last fragment: message is complete ────────────────────────────────────
    if (s_frag.received == s_frag.total_len) {
      s_frag.buf[s_frag.total_len] = '\0';
      ESP_LOGI(TAG, "Config message complete: %d bytes — dispatching to callback",
               s_frag.total_len);
      if (s_config_cb) {
        s_config_cb(s_frag.buf);
      } else {
        ESP_LOGW(TAG, "Config received but no callback registered");
      }
      free(s_frag.buf);
      s_frag.buf = NULL;
      s_frag.total_len = 0;
      s_frag.received = 0;
    }
    break;

  default:
    ESP_LOGD(TAG, "Unhandled MQTT event: %d", (int)event_id);
    break;
  }
}

esp_err_t mqtt_client_mgr_init(const mqtt_client_config_t *config) {
  if (s_mqtt.initialized) {
    ESP_LOGW(TAG, "MQTT client manager already initialized");
    return ESP_OK;
  }

  // If config is NULL, load from NVS
  nvs_device_config_t nvs_snap;
  const nvs_device_config_t *nvs_cfg = nvs_config_snapshot(&nvs_snap) ? &nvs_snap : NULL;

  // If config is NULL, load from NVS
  if (config == NULL) {
    if (nvs_cfg == NULL) {
      ESP_LOGE(TAG, "No configuration available in NVS");
      return ESP_ERR_NOT_FOUND;
    }
    strncpy(s_mqtt.config.broker_host, nvs_cfg->mqtt_broker,
            sizeof(s_mqtt.config.broker_host) - 1);
    s_mqtt.config.broker_port = nvs_cfg->mqtt_port;
    strncpy(s_mqtt.config.username, nvs_cfg->mqtt_username,
            sizeof(s_mqtt.config.username) - 1);
    strncpy(s_mqtt.config.password, nvs_cfg->mqtt_password,
            sizeof(s_mqtt.config.password) - 1);
    // Use device_id as client_id
    strncpy(s_mqtt.config.client_id,
            nvs_cfg->device_id,
            sizeof(s_mqtt.config.client_id) - 1);
    // Build base topic
    snprintf(s_mqtt.config.base_topic, sizeof(s_mqtt.config.base_topic),
             "Limelight/factory/%s", s_mqtt.config.client_id);
    ESP_LOGI(TAG, "MQTT config loaded from NVS");
  } else {
    // Copy configuration from parameter
    memcpy(&s_mqtt.config, config, sizeof(mqtt_client_config_t));
  }

  // Build topic strings
  snprintf(s_mqtt.state_topic, sizeof(s_mqtt.state_topic), "%s/statechange",
           s_mqtt.config.base_topic);
  snprintf(s_mqtt.livedata_topic, sizeof(s_mqtt.livedata_topic), "%s/livedata",
           s_mqtt.config.base_topic);
  snprintf(s_mqtt.lwt_topic, sizeof(s_mqtt.lwt_topic), "%s/lwt",
           s_mqtt.config.base_topic);
  snprintf(s_mqtt.config_topic, sizeof(s_mqtt.config_topic), "%s/config",
           s_mqtt.config.base_topic);
  snprintf(s_mqtt.config_ack_topic, sizeof(s_mqtt.config_ack_topic),
           "%s/config/ack", s_mqtt.config.base_topic);

  // Build broker URI. Accept either "host", "host:port", or "mqtt://host:port"
  // from provisioning, so field-entry differences do not create invalid URIs.
  char broker_uri[160];
  esp_err_t uri_ret =
      build_broker_uri(broker_uri, sizeof(broker_uri), &s_mqtt.config);
  if (uri_ret != ESP_OK) {
    return uri_ret;
  }
  bool broker_uses_tls = strncmp(broker_uri, "mqtts://", 8) == 0;
  if (!broker_uses_tls) {
    ESP_LOGW(TAG, "MQTT transport is plaintext; use mqtts:// in production");
  }

  int keepalive = DEFAULT_MQTT_KEEPALIVE;
  if (nvs_cfg) {
    keepalive = nvs_cfg->mqtt_keepalive;
  }

  // Configure MQTT client with keep-alive and LWT
  esp_mqtt_client_config_t mqtt_cfg = {
      .broker.address.uri = broker_uri,
      .broker.verification.crt_bundle_attach =
          broker_uses_tls ? esp_crt_bundle_attach : NULL,
      .credentials.client_id = s_mqtt.config.client_id,
      .credentials.username = s_mqtt.config.username,
      .credentials.authentication.password = s_mqtt.config.password,
      .session.keepalive = keepalive,
      .session.last_will =
          {
              .topic = s_mqtt.lwt_topic,
              .msg = MQTT_LWT_MSG_OFFLINE,
              .msg_len = sizeof(MQTT_LWT_MSG_OFFLINE) - 1,
              .qos = MQTT_LWT_QOS,
              .retain = MQTT_LWT_RETAIN,
          },
      .network.reconnect_timeout_ms = MQTT_RECONNECT_TIMEOUT_MS,
      .buffer.size = MQTT_BUFFER_SIZE,
  };

  ESP_LOGI(TAG, "MQTT Keep-alive: %d sec, LWT topic: %s", keepalive,
           s_mqtt.lwt_topic);

  // Create MQTT client
  s_mqtt.client = esp_mqtt_client_init(&mqtt_cfg);
  if (s_mqtt.client == NULL) {
    ESP_LOGE(TAG, "Failed to create MQTT client");
    return ESP_FAIL;
  }

  // Register event handler
  esp_err_t ret = esp_mqtt_client_register_event(
      s_mqtt.client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to register MQTT event handler: %s",
             esp_err_to_name(ret));
    esp_mqtt_client_destroy(s_mqtt.client);
    s_mqtt.client = NULL;
    return ret;
  }

  // Initialize status
  s_mqtt.status.connected = false;
  s_mqtt.status.messages_sent = 0;
  s_mqtt.status.messages_failed = 0;

  // #ISSUE-39: mutex protecting s_mqtt.status from cross-core races
  s_mqtt.status_mutex = xSemaphoreCreateMutex();
  if (s_mqtt.status_mutex == NULL) {
    ESP_LOGE(TAG, "Failed to create status_mutex");
    esp_mqtt_client_destroy(s_mqtt.client);
    s_mqtt.client = NULL;
    return ESP_ERR_NO_MEM;
  }

  s_mqtt.initialized = true;

  ESP_LOGI(TAG, "MQTT client initialized, uri: %s, client_id: %s",
           broker_uri, s_mqtt.config.client_id);
  ESP_LOGI(TAG, "Topics - State: %s", s_mqtt.state_topic);
  ESP_LOGI(TAG, "Topics - Livedata: %s", s_mqtt.livedata_topic);
  ESP_LOGI(TAG, "Topics - LWT: %s", s_mqtt.lwt_topic);

  return ESP_OK;
}

esp_err_t mqtt_client_mgr_start(void) {
  if (!s_mqtt.initialized) {
    ESP_LOGE(TAG, "MQTT client not initialized");
    return ESP_ERR_INVALID_STATE;
  }

  if (s_mqtt.started) {
    ESP_LOGW(TAG, "MQTT client already started");
    return ESP_OK;
  }

  esp_err_t ret = esp_mqtt_client_start(s_mqtt.client);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to start MQTT client: %s", esp_err_to_name(ret));
    return ret;
  }

  s_mqtt.started = true;
  ESP_LOGI(TAG, "MQTT client started");

  return ESP_OK;
}

void mqtt_client_mgr_stop(void) {
  if (!s_mqtt.initialized || !s_mqtt.started) {
    return;
  }

  esp_mqtt_client_stop(s_mqtt.client);
  s_mqtt.started = false;
  if (xSemaphoreTake(s_mqtt.status_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    s_mqtt.status.connected = false;
    xSemaphoreGive(s_mqtt.status_mutex);
  }

  ESP_LOGI(TAG, "MQTT client stopped");
}

bool mqtt_client_mgr_is_connected(void) {
  if (s_mqtt.status_mutex == NULL) return false;
  if (xSemaphoreTake(s_mqtt.status_mutex, pdMS_TO_TICKS(50)) != pdTRUE) {
    ESP_LOGW(TAG, "status_mutex timeout in is_connected — returning false");
    return false;
  }
  bool connected = s_mqtt.status.connected;
  xSemaphoreGive(s_mqtt.status_mutex);
  return connected;
}

void mqtt_client_mgr_get_status(mqtt_status_t *status) {
  if (status == NULL) return;
  if (s_mqtt.status_mutex == NULL) {
    memset(status, 0, sizeof(mqtt_status_t));
    return;
  }
  if (xSemaphoreTake(s_mqtt.status_mutex, pdMS_TO_TICKS(50)) != pdTRUE) {
    ESP_LOGW(TAG, "status_mutex timeout in get_status — returning zeroed struct");
    memset(status, 0, sizeof(mqtt_status_t));
    return;
  }
  memcpy(status, &s_mqtt.status, sizeof(mqtt_status_t));
  xSemaphoreGive(s_mqtt.status_mutex);
}

esp_err_t mqtt_client_mgr_publish(const char *topic, const char *payload,
                                  int qos, bool retain) {
  if (!s_mqtt.initialized) {
    ESP_LOGE(TAG, "MQTT client not initialized");
    return ESP_ERR_INVALID_STATE;
  }

  if (topic == NULL || payload == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  if (xSemaphoreTake(s_mqtt.status_mutex, pdMS_TO_TICKS(50)) != pdTRUE) {
    ESP_LOGW(TAG, "status_mutex timeout checking connection — skipping publish");
    return ESP_FAIL;
  }
  if (!s_mqtt.status.connected) {
    ESP_LOGD(TAG, "MQTT not connected, message not sent");
    s_mqtt.status.messages_failed++;
    xSemaphoreGive(s_mqtt.status_mutex);
    return ESP_FAIL;
  }
  xSemaphoreGive(s_mqtt.status_mutex);

  int msg_id = esp_mqtt_client_publish(s_mqtt.client, topic, payload,
                                       strlen(payload), qos, retain ? 1 : 0);

  if (xSemaphoreTake(s_mqtt.status_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    if (msg_id < 0) {
      s_mqtt.status.messages_failed++;
    } else {
      s_mqtt.status.messages_sent++;
    }
    xSemaphoreGive(s_mqtt.status_mutex);
  } else {
    ESP_LOGW(TAG, "status_mutex timeout updating publish stats — counts may drift");
  }

  if (msg_id < 0) {
    ESP_LOGE(TAG, "Failed to publish to %s", topic);
    return ESP_FAIL;
  }

  ESP_LOGD(TAG, "Published to %s (msg_id=%d, qos=%d)", topic, msg_id, qos);

  return ESP_OK;
}

esp_err_t mqtt_client_mgr_publish_state(const char *json_payload) {
  if (json_payload == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  // Use QoS 1 for state messages (important, need delivery confirmation)
  return mqtt_client_mgr_publish(s_mqtt.state_topic, json_payload, 1, false);
}

esp_err_t mqtt_client_mgr_publish_livedata(const char *json_payload) {
  if (json_payload == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  // Use QoS 0 for livedata (high frequency, fire-and-forget)
  return mqtt_client_mgr_publish(s_mqtt.livedata_topic, json_payload, 0, false);
}

void mqtt_client_mgr_set_config_callback(mqtt_config_cb_t cb) {
  s_config_cb = cb;
  ESP_LOGI(TAG, "Config callback %s", cb ? "registered" : "cleared");
}

esp_err_t mqtt_client_mgr_publish_config_ack(const char *config_id,
                                             const char *catalog_version,
                                             const char *status,
                                             const char *error) {
  if (!s_mqtt.initialized) {
    return ESP_ERR_INVALID_STATE;
  }
  if (xSemaphoreTake(s_mqtt.status_mutex, pdMS_TO_TICKS(50)) != pdTRUE) {
    ESP_LOGW(TAG, "status_mutex timeout in publish_config_ack — skipping");
    return ESP_ERR_INVALID_STATE;
  }
  bool connected = s_mqtt.status.connected;
  xSemaphoreGive(s_mqtt.status_mutex);
  if (!connected) {
    return ESP_ERR_INVALID_STATE;
  }
  if (config_id == NULL || status == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  cJSON *root = cJSON_CreateObject();
  if (!root) {
    return ESP_ERR_NO_MEM;
  }

  cJSON_AddStringToObject(root, "msg_type", "config.ack");
  cJSON_AddStringToObject(root, "config_id", config_id);
  cJSON_AddStringToObject(root, "catalog_version",
                          catalog_version ? catalog_version : "");
  cJSON_AddStringToObject(root, "status", status);

  // Add ISO8601 timestamp
  time_t now;
  struct tm timeinfo;
  time(&now);
  localtime_r(&now, &timeinfo);
  char timestamp[32];
  if (timeinfo.tm_year > (2020 - 1900)) {
    strftime(timestamp, sizeof(timestamp), "%Y-%m-%dT%H:%M:%S.000Z", &timeinfo);
  } else {
    snprintf(timestamp, sizeof(timestamp), "unknown");
  }
  cJSON_AddStringToObject(root, "ts", timestamp);

  if (error) {
    cJSON_AddStringToObject(root, "error", error);
  }

  char *json_str = cJSON_PrintUnformatted(root);
  cJSON_Delete(root);

  if (!json_str) {
    return ESP_ERR_NO_MEM;
  }

  // Publish with QoS 1 (guaranteed delivery for ACK)
  esp_err_t ret =
      mqtt_client_mgr_publish(s_mqtt.config_ack_topic, json_str, 1, false);

  ESP_LOGI(TAG, "Config ACK sent: status=%s, config_id=%s", status, config_id);

  cJSON_free(json_str);
  return ret;
}
