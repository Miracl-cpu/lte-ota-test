/**
 * @file mqtt_client_mgr.h
 * @brief MQTT Client Manager for CT+HMI System
 *
 * Provides MQTT client connectivity with:
 * - Automatic reconnection on disconnect
 * - Topic-based publishing (state and livedata)
 * - Message statistics tracking
 * - QoS 1 for state messages, QoS 0 for livedata
 */

#ifndef MQTT_CLIENT_MGR_H
#define MQTT_CLIENT_MGR_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief MQTT connection status
 */
typedef struct {
    bool connected;
    uint32_t messages_sent;
    uint32_t messages_failed;
} mqtt_status_t;

/**
 * @brief MQTT configuration
 */
typedef struct {
    char broker_host[64];
    uint16_t broker_port;
    char client_id[32];
    char username[32];
    char password[64];
    char base_topic[64];      // e.g., "Limelight/factory/DM-002"
} mqtt_client_config_t;

/**
 * @brief Initialize MQTT client (does not connect yet)
 * @param config MQTT configuration
 * @return ESP_OK on success
 */
esp_err_t mqtt_client_mgr_init(const mqtt_client_config_t *config);

/**
 * @brief Start MQTT client (call after WiFi connected)
 * @return ESP_OK on success
 */
esp_err_t mqtt_client_mgr_start(void);

/**
 * @brief Stop MQTT client
 */
void mqtt_client_mgr_stop(void);

/**
 * @brief Check if MQTT is connected
 * @return true if connected to broker
 */
bool mqtt_client_mgr_is_connected(void);

/**
 * @brief Get MQTT status
 * @param status Output status structure
 */
void mqtt_client_mgr_get_status(mqtt_status_t *status);

/**
 * @brief Publish to state topic (Limelight/factory/{client_id}/statechange)
 * @param json_payload JSON string to publish
 * @return ESP_OK on success, ESP_FAIL if not connected
 */
esp_err_t mqtt_client_mgr_publish_state(const char *json_payload);

/**
 * @brief Publish to livedata topic (Limelight/factory/{client_id}/livedata)
 * @param json_payload JSON string to publish
 * @return ESP_OK on success, ESP_FAIL if not connected
 */
esp_err_t mqtt_client_mgr_publish_livedata(const char *json_payload);

/**
 * @brief Generic publish function
 * @param topic Full topic string
 * @param payload Payload string
 * @param qos QoS level (0, 1, or 2)
 * @param retain Retain flag
 * @return ESP_OK on success
 */
esp_err_t mqtt_client_mgr_publish(const char *topic, const char *payload, int qos, bool retain);

/**
 * @brief Config message callback type
 * @param config_json JSON configuration string (null-terminated)
 */
typedef void (*mqtt_config_cb_t)(const char *config_json);

/**
 * @brief Register callback for incoming config messages
 * @param cb Function to call when config is received on .../config topic
 */
void mqtt_client_mgr_set_config_callback(mqtt_config_cb_t cb);

/**
 * @brief Publish config ACK message
 * @param config_id Config ID being acknowledged
 * @param catalog_version Catalog version applied (can be NULL)
 * @param status "applied" or "rejected"
 * @param error Optional error message (NULL for success)
 * @return ESP_OK on success
 */
esp_err_t mqtt_client_mgr_publish_config_ack(const char *config_id,
                                              const char *catalog_version,
                                              const char *status,
                                              const char *error);

#ifdef __cplusplus
}
#endif

#endif // MQTT_CLIENT_MGR_H
