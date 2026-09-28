/**
 * @file setup_webserver.c
 * @brief HTTP web server for configuration UI implementation
 *
 * HTTP Endpoints:
 * - GET  /               Main configuration page (HTML)
 * - GET  /generate_204   Android captive portal detection
 * - GET  /hotspot-detect.html  iOS captive portal detection
 * - GET  /connecttest.txt      Windows captive portal detection
 * - GET  /ncsi.txt             Windows captive portal detection
 * - GET  /redirect             Windows captive portal detection
 * - GET  /fwlink               Windows captive portal detection
 * - GET  /scan           Trigger WiFi scan, return JSON
 * - GET  /config         Get current config as JSON
 * - POST /config         Save config (JSON body)
 * - POST /reboot         Trigger reboot
 * - GET  /status         Get device status JSON
 */

#include "setup_webserver.h"
#include "setup_trigger.h"
#include "web_content.h"
#include "nvs_config.h"
#include "esp_http_server.h"
#include "esp_wifi.h"
#include "log_config.h"
#undef LOG_LOCAL_LEVEL
#define LOG_LOCAL_LEVEL LOG_LEVEL_SETUP_WEBSERVER
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_timer.h"
#include "cJSON.h"
#include "ota_drive.h"
#include <string.h>
#include <strings.h>

static const char *TAG = "setup_webserver";

// Internal state
static httpd_handle_t s_server = NULL;
static webserver_config_saved_cb_t s_config_saved_cb = NULL;
static bool s_active = false;
static bool s_scan_in_progress = false;

/**
 * @brief Send JSON response
 */
static esp_err_t send_json_response(httpd_req_t *req, cJSON *json)
{
    char *json_str = cJSON_PrintUnformatted(json);
    if (json_str == NULL) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_send(req, json_str, strlen(json_str));

    free(json_str);
    return ESP_OK;
}

/**
 * @brief Handler for GET / - Main configuration page
 */
static esp_err_t index_get_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "GET /");
    setup_trigger_reset_activity((uint32_t)(esp_timer_get_time() / 1000));
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, web_content_get_index_html(), web_content_get_index_html_len());
    return ESP_OK;
}

/**
 * @brief Handler for captive portal detection (Android)
 */
static esp_err_t generate_204_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "GET /generate_204 (Android captive portal)");
    // Redirect to config page
    httpd_resp_set_status(req, "302 Found");
    httpd_resp_set_hdr(req, "Location", "http://192.168.4.1/");
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
}

/**
 * @brief Handler for captive portal detection (iOS)
 */
static esp_err_t hotspot_detect_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "GET /hotspot-detect.html (iOS captive portal)");
    // Redirect to config page
    httpd_resp_set_status(req, "302 Found");
    httpd_resp_set_hdr(req, "Location", "http://192.168.4.1/");
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
}

/**
 * @brief Handler for captive portal detection (Windows)
 * Windows uses several endpoints: /connecttest.txt, /ncsi.txt, /redirect, /fwlink
 */
static esp_err_t windows_captive_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "GET %s (Windows captive portal)", req->uri);
    // Redirect to config page
    httpd_resp_set_status(req, "302 Found");
    httpd_resp_set_hdr(req, "Location", "http://192.168.4.1/");
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
}

/**
 * @brief Handler for GET /scan - WiFi network scan
 */
static esp_err_t scan_get_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "GET /scan - Starting WiFi scan");
    setup_trigger_reset_activity((uint32_t)(esp_timer_get_time() / 1000));

    // Scan concurrency guard
    if (s_scan_in_progress) {
        ESP_LOGW(TAG, "Scan already in progress");
        cJSON *error = cJSON_CreateObject();
        cJSON_AddStringToObject(error, "error", "scan_busy");
        httpd_resp_set_status(req, "409 Conflict");
        send_json_response(req, error);
        cJSON_Delete(error);
        return ESP_OK;
    }
    s_scan_in_progress = true;

    // Verify WiFi mode includes STA (required for scanning)
    bool upgraded_to_apsta = false;
    wifi_mode_t mode;
    esp_err_t ret = esp_wifi_get_mode(&mode);
    if (ret == ESP_OK && mode == WIFI_MODE_AP) {
        ESP_LOGI(TAG, "Upgrading WiFi mode from AP to APSTA for scanning");
        ret = esp_wifi_set_mode(WIFI_MODE_APSTA);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to set APSTA mode: %s", esp_err_to_name(ret));
            s_scan_in_progress = false;
            cJSON *error = cJSON_CreateObject();
            cJSON_AddStringToObject(error, "error", "Failed to enable scan mode");
            send_json_response(req, error);
            cJSON_Delete(error);
            return ESP_OK;
        }
        upgraded_to_apsta = true;
    }

    // Configure and start scan
    wifi_scan_config_t scan_config = {
        .ssid = NULL,
        .bssid = NULL,
        .channel = 0,
        .show_hidden = false,
        .scan_type = WIFI_SCAN_TYPE_ACTIVE,
        .scan_time.active.min = 100,
        .scan_time.active.max = 300,
    };

    ret = esp_wifi_scan_start(&scan_config, true);  // Blocking scan
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_scan_start failed: %s", esp_err_to_name(ret));
        if (upgraded_to_apsta) {
            esp_wifi_set_mode(WIFI_MODE_AP);  // Revert before early exit
        }
        s_scan_in_progress = false;
        cJSON *error = cJSON_CreateObject();
        cJSON_AddStringToObject(error, "error", "Scan failed");
        send_json_response(req, error);
        cJSON_Delete(error);
        return ESP_OK;
    }

    // Get scan results
    uint16_t ap_count = 0;
    ret = esp_wifi_scan_get_ap_num(&ap_count);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_scan_get_ap_num failed: %s", esp_err_to_name(ret));
    }

    wifi_ap_record_t *ap_records = NULL;
    if (ap_count > 0) {
        ap_records = malloc(sizeof(wifi_ap_record_t) * ap_count);
        if (ap_records) {
            ret = esp_wifi_scan_get_ap_records(&ap_count, ap_records);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "esp_wifi_scan_get_ap_records failed: %s", esp_err_to_name(ret));
            }
        }
    }

    // Build JSON response
    cJSON *response = cJSON_CreateObject();
    cJSON *networks = cJSON_CreateArray();

    for (int i = 0; i < ap_count && i < 20; i++) {  // Limit to 20 networks
        cJSON *network = cJSON_CreateObject();
        cJSON_AddStringToObject(network, "ssid", (char *)ap_records[i].ssid);
        cJSON_AddNumberToObject(network, "rssi", ap_records[i].rssi);

        // Auth mode string
        const char *auth = "UNKNOWN";
        switch (ap_records[i].authmode) {
            case WIFI_AUTH_OPEN:            auth = "OPEN"; break;
            case WIFI_AUTH_WEP:             auth = "WEP"; break;
            case WIFI_AUTH_WPA_PSK:         auth = "WPA"; break;
            case WIFI_AUTH_WPA2_PSK:        auth = "WPA2"; break;
            case WIFI_AUTH_WPA_WPA2_PSK:    auth = "WPA/WPA2"; break;
            case WIFI_AUTH_WPA3_PSK:        auth = "WPA3"; break;
            case WIFI_AUTH_WPA2_WPA3_PSK:   auth = "WPA2/WPA3"; break;
            default: break;
        }
        cJSON_AddStringToObject(network, "auth", auth);
        cJSON_AddItemToArray(networks, network);
    }

    cJSON_AddItemToObject(response, "networks", networks);

    if (ap_records) {
        free(ap_records);
    }

    // Revert WiFi mode back to AP-only if we upgraded it for this scan
    if (upgraded_to_apsta) {
        esp_err_t revert_ret = esp_wifi_set_mode(WIFI_MODE_AP);
        if (revert_ret == ESP_OK) {
            ESP_LOGI(TAG, "WiFi mode reverted APSTA -> AP after scan");
        } else {
            ESP_LOGW(TAG, "Failed to revert WiFi mode after scan: %s", esp_err_to_name(revert_ret));
        }
    }

    s_scan_in_progress = false;
    ESP_LOGI(TAG, "Scan complete: %d networks found", ap_count);
    send_json_response(req, response);
    cJSON_Delete(response);

    return ESP_OK;
}

/**
 * @brief Handler for GET /config - Get current configuration
 */
static esp_err_t config_get_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "GET /config");
    setup_trigger_reset_activity((uint32_t)(esp_timer_get_time() / 1000));

    nvs_device_config_t cfg_snap;
    const nvs_device_config_t *cfg = nvs_config_snapshot(&cfg_snap) ? &cfg_snap : NULL;

    cJSON *response = cJSON_CreateObject();

    // Device identity
    cJSON_AddStringToObject(response, "device_id", cfg ? cfg->device_id : "");
    cJSON_AddStringToObject(response, "location", cfg ? cfg->location : "");

    // WiFi
    cJSON_AddStringToObject(response, "wifi_ssid", cfg ? cfg->wifi_ssid : "");
    cJSON_AddBoolToObject(response, "wifi_configured", cfg ? cfg->wifi_configured : false);

    // MQTT
    cJSON_AddStringToObject(response, "mqtt_broker", cfg ? cfg->mqtt_broker : DEFAULT_MQTT_BROKER);
    cJSON_AddNumberToObject(response, "mqtt_port", cfg ? cfg->mqtt_port : DEFAULT_MQTT_PORT);
    cJSON_AddStringToObject(response, "mqtt_username", cfg ? cfg->mqtt_username : DEFAULT_MQTT_USERNAME);

    // CT calibration
    cJSON *ct_gain = cJSON_CreateArray();
    cJSON_AddItemToArray(ct_gain, cJSON_CreateNumber(cfg ? cfg->ct_gain[0] : DEFAULT_CT_GAIN));
    cJSON_AddItemToArray(ct_gain, cJSON_CreateNumber(cfg ? cfg->ct_gain[1] : DEFAULT_CT_GAIN));
    cJSON_AddItemToArray(ct_gain, cJSON_CreateNumber(cfg ? cfg->ct_gain[2] : DEFAULT_CT_GAIN));
    cJSON_AddItemToObject(response, "ct_gain", ct_gain);

    // Machine thresholds
    cJSON_AddNumberToObject(response, "active_threshold", cfg ? cfg->active_threshold : DEFAULT_ACTIVE_THR);
    cJSON_AddNumberToObject(response, "hysteresis_factor", cfg ? cfg->hysteresis_factor : DEFAULT_HYSTERESIS);
    cJSON_AddNumberToObject(response, "active_sustain_ms", cfg ? cfg->active_sustain_ms : DEFAULT_SUSTAIN_MS);
    cJSON_AddNumberToObject(response, "machine_setup_time_ms", cfg ? cfg->machine_setup_time_ms : DEFAULT_MACHINE_SETUP_MS);
    cJSON_AddNumberToObject(response, "voltage_nominal", cfg ? cfg->voltage_nominal : DEFAULT_VOLTAGE_NOMINAL);
    cJSON_AddNumberToObject(response, "power_factor", cfg ? cfg->power_factor : DEFAULT_POWER_FACTOR);

    // CT & Input Sources
    cJSON_AddNumberToObject(response, "ct_enabled_mask", cfg ? cfg->ct_enabled_mask : 0x07);
    cJSON_AddBoolToObject(response, "ignore_open_ct", cfg ? cfg->ignore_open_ct : true);
    cJSON_AddNumberToObject(response, "state_source", cfg ? (int)cfg->state_source : 0);
    cJSON_AddStringToObject(response, "state_detection",
                            (cfg && cfg->state_source == NVS_STATE_SRC_SIGNAL)
                                ? "signal_based"
                                : "ct");
    cJSON_AddNumberToObject(response, "signal_logic", cfg ? (int)cfg->signal_logic : 0);
    cJSON_AddStringToObject(response, "signal_logic_text",
                            (cfg && cfg->signal_logic == NVS_SIGNAL_LOGIC_NC)
                                ? "NC"
                                : "NO");
    cJSON_AddNumberToObject(response, "state_ct_phase_mode", cfg ? (int)cfg->state_ct_phase_mode : 1);
    cJSON_AddNumberToObject(response, "energy_ct_phase_mode", cfg ? (int)cfg->energy_ct_phase_mode : 1);
    cJSON_AddNumberToObject(response, "state_idle_stabilization_ms",
                            cfg ? cfg->state_idle_stabilization_ms : DEFAULT_STATE_IDLE_STABILIZATION_MS);
    cJSON_AddNumberToObject(response, "proxy_enabled_mask", cfg ? cfg->proxy_enabled_mask : 0x01);
    cJSON_AddBoolToObject(response, "encoder_enabled", cfg ? cfg->encoder_enabled : true);
    cJSON_AddNumberToObject(response, "encoder_ppr", cfg ? cfg->encoder_ppr : DEFAULT_ENC_PPR);
    cJSON_AddNumberToObject(response, "encoder_mpm_factor", cfg ? cfg->encoder_mpm_factor : DEFAULT_ENC_MPM_FACTOR);

    // Communication
    cJSON_AddNumberToObject(response, "uplink_mode", cfg ? (int)cfg->uplink_mode : 0);
    cJSON_AddNumberToObject(response, "mqtt_keepalive", cfg ? cfg->mqtt_keepalive : DEFAULT_MQTT_KEEPALIVE);
    cJSON_AddNumberToObject(response, "livedata_interval_ms", cfg ? cfg->livedata_interval_ms : DEFAULT_LIVEDATA_INT);
    cJSON_AddStringToObject(response, "softap_ssid_prefix", cfg ? cfg->softap_ssid_prefix : DEFAULT_SSID_PREFIX);

    // System info
    cJSON_AddBoolToObject(response, "provisioned", cfg ? cfg->provisioned : false);

    // MAC address
    char mac_str[18];
    nvs_config_get_mac_string(mac_str, sizeof(mac_str));
    cJSON_AddStringToObject(response, "mac_address", mac_str);

    // Firmware version
    cJSON_AddStringToObject(response, "firmware_version", ota_drive_get_version());

    send_json_response(req, response);
    cJSON_Delete(response);

    return ESP_OK;
}

/**
 * @brief Handler for POST /config - Save configuration
 */
static esp_err_t config_post_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "POST /config - Saving configuration");
    setup_trigger_reset_activity((uint32_t)(esp_timer_get_time() / 1000));

    // Read request body
    int content_len = req->content_len;
    if (content_len <= 0 || content_len > 2048) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid content length");
        return ESP_OK;
    }

    char *body = malloc(content_len + 1);
    if (body == NULL) {
        httpd_resp_send_500(req);
        return ESP_OK;
    }

    int received = httpd_req_recv(req, body, content_len);
    if (received != content_len) {
        free(body);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Failed to receive body");
        return ESP_OK;
    }
    body[content_len] = '\0';

    ESP_LOGI(TAG, "Received config: %s", body);

    // Parse JSON
    cJSON *json = cJSON_Parse(body);
    free(body);

    if (json == NULL) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_OK;
    }

    // Load current config and update with new values
    nvs_device_config_t new_cfg;
    if (!nvs_config_snapshot(&new_cfg)) {
        nvs_config_load_defaults(&new_cfg);
    }

    // Update device identity
    cJSON *item = cJSON_GetObjectItem(json, "device_id");
    if (item && cJSON_IsString(item)) {
        strncpy(new_cfg.device_id, item->valuestring, NVS_DEVICE_ID_LEN - 1);
        new_cfg.device_id[NVS_DEVICE_ID_LEN - 1] = '\0';
    }

    item = cJSON_GetObjectItem(json, "location");
    if (item && cJSON_IsString(item)) {
        strncpy(new_cfg.location, item->valuestring, NVS_LOCATION_LEN - 1);
        new_cfg.location[NVS_LOCATION_LEN - 1] = '\0';
    }

    // Update WiFi credentials
    item = cJSON_GetObjectItem(json, "wifi_ssid");
    if (item && cJSON_IsString(item)) {
        strncpy(new_cfg.wifi_ssid, item->valuestring, NVS_WIFI_SSID_LEN - 1);
        new_cfg.wifi_ssid[NVS_WIFI_SSID_LEN - 1] = '\0';
    }

    item = cJSON_GetObjectItem(json, "wifi_password");
    if (item && cJSON_IsString(item)) {
        strncpy(new_cfg.wifi_password, item->valuestring, NVS_WIFI_PASS_LEN - 1);
        new_cfg.wifi_password[NVS_WIFI_PASS_LEN - 1] = '\0';
    }

    // Mark WiFi as configured if SSID is set
    new_cfg.wifi_configured = strlen(new_cfg.wifi_ssid) > 0;

    // Update MQTT settings
    item = cJSON_GetObjectItem(json, "mqtt_broker");
    if (item && cJSON_IsString(item)) {
        strncpy(new_cfg.mqtt_broker, item->valuestring, NVS_MQTT_HOST_LEN - 1);
        new_cfg.mqtt_broker[NVS_MQTT_HOST_LEN - 1] = '\0';
    }

    item = cJSON_GetObjectItem(json, "mqtt_port");
    if (item && cJSON_IsNumber(item)) {
        new_cfg.mqtt_port = (uint16_t)item->valueint;
    }

    item = cJSON_GetObjectItem(json, "mqtt_username");
    if (item && cJSON_IsString(item)) {
        strncpy(new_cfg.mqtt_username, item->valuestring, NVS_MQTT_USER_LEN - 1);
        new_cfg.mqtt_username[NVS_MQTT_USER_LEN - 1] = '\0';
    }

    item = cJSON_GetObjectItem(json, "mqtt_password");
    if (item && cJSON_IsString(item)) {
        strncpy(new_cfg.mqtt_password, item->valuestring, NVS_MQTT_PASS_LEN - 1);
        new_cfg.mqtt_password[NVS_MQTT_PASS_LEN - 1] = '\0';
    }

    // Update CT calibration
    item = cJSON_GetObjectItem(json, "ct_gain");
    if (item && cJSON_IsArray(item)) {
        for (int i = 0; i < 3 && i < cJSON_GetArraySize(item); i++) {
            cJSON *gain = cJSON_GetArrayItem(item, i);
            if (cJSON_IsNumber(gain)) {
                new_cfg.ct_gain[i] = (float)gain->valuedouble;
            }
        }
    }

    // Update threshold
    item = cJSON_GetObjectItem(json, "active_threshold");
    if (item && cJSON_IsNumber(item)) {
        new_cfg.active_threshold = (float)item->valuedouble;
    }

    // Machine settings
    item = cJSON_GetObjectItem(json, "hysteresis_factor");
    if (item && cJSON_IsNumber(item)) {
        new_cfg.hysteresis_factor = (float)item->valuedouble;
    }

    item = cJSON_GetObjectItem(json, "active_sustain_ms");
    if (item && cJSON_IsNumber(item)) {
        new_cfg.active_sustain_ms = (uint32_t)item->valueint;
    }

    item = cJSON_GetObjectItem(json, "machine_setup_time_ms");
    if (item && cJSON_IsNumber(item)) {
        new_cfg.machine_setup_time_ms = (uint32_t)item->valueint;
    }

    item = cJSON_GetObjectItem(json, "voltage_nominal");
    if (item && cJSON_IsNumber(item)) {
        new_cfg.voltage_nominal = (float)item->valuedouble;
    }

    item = cJSON_GetObjectItem(json, "power_factor");
    if (item && cJSON_IsNumber(item)) {
        new_cfg.power_factor = (float)item->valuedouble;
    }

    // CT & Input Sources
    item = cJSON_GetObjectItem(json, "ct_enabled_mask");
    if (item && cJSON_IsNumber(item)) {
        new_cfg.ct_enabled_mask = (uint8_t)item->valueint;
    }

    item = cJSON_GetObjectItem(json, "ignore_open_ct");
    if (item && cJSON_IsBool(item)) {
        new_cfg.ignore_open_ct = cJSON_IsTrue(item);
    }

    item = cJSON_GetObjectItem(json, "state_source");
    if (item && cJSON_IsNumber(item)) {
        new_cfg.state_source = (item->valueint == (int)NVS_STATE_SRC_SIGNAL)
                                   ? NVS_STATE_SRC_SIGNAL
                                   : NVS_STATE_SRC_CT;
    }
    item = cJSON_GetObjectItem(json, "state_detection");
    if (item && cJSON_IsString(item)) {
        if (strcasecmp(item->valuestring, "signal_based") == 0 ||
            strcasecmp(item->valuestring, "signal") == 0) {
            new_cfg.state_source = NVS_STATE_SRC_SIGNAL;
        } else if (strcasecmp(item->valuestring, "ct") == 0) {
            new_cfg.state_source = NVS_STATE_SRC_CT;
        }
    }

    item = cJSON_GetObjectItem(json, "signal_logic");
    if (item && cJSON_IsNumber(item)) {
        new_cfg.signal_logic = (item->valueint == (int)NVS_SIGNAL_LOGIC_NC)
                                   ? NVS_SIGNAL_LOGIC_NC
                                   : NVS_SIGNAL_LOGIC_NO;
    }
    if (item && cJSON_IsString(item)) {
        if (strcasecmp(item->valuestring, "nc") == 0) {
            new_cfg.signal_logic = NVS_SIGNAL_LOGIC_NC;
        } else if (strcasecmp(item->valuestring, "no") == 0) {
            new_cfg.signal_logic = NVS_SIGNAL_LOGIC_NO;
        }
    }

    item = cJSON_GetObjectItem(json, "state_ct_phase_mode");
    if (item && cJSON_IsNumber(item)) {
        new_cfg.state_ct_phase_mode = (nvs_ct_phase_mode_t)item->valueint;
    }

    item = cJSON_GetObjectItem(json, "energy_ct_phase_mode");
    if (item && cJSON_IsNumber(item)) {
        new_cfg.energy_ct_phase_mode = (nvs_ct_phase_mode_t)item->valueint;
        new_cfg.ct_phase_mode = new_cfg.energy_ct_phase_mode;
    }

    item = cJSON_GetObjectItem(json, "state_idle_stabilization_ms");
    if (item && cJSON_IsNumber(item)) {
        new_cfg.state_idle_stabilization_ms = (uint32_t)item->valueint;
    }

    item = cJSON_GetObjectItem(json, "proxy_enabled_mask");
    if (item && cJSON_IsNumber(item)) {
        uint8_t requested_mask = (uint8_t)item->valueint;
        new_cfg.proxy_enabled_mask =
            (requested_mask & 0x01) ? 0x01
            : (requested_mask & 0x02) ? 0x02 : 0x01;
    }

    item = cJSON_GetObjectItem(json, "encoder_enabled");
    if (item && cJSON_IsBool(item)) {
        new_cfg.encoder_enabled = cJSON_IsTrue(item);
    }

    item = cJSON_GetObjectItem(json, "encoder_ppr");
    if (item && cJSON_IsNumber(item)) {
        new_cfg.encoder_ppr = (float)item->valuedouble;
    }

    item = cJSON_GetObjectItem(json, "encoder_mpm_factor");
    if (item && cJSON_IsNumber(item)) {
        new_cfg.encoder_mpm_factor = (float)item->valuedouble;
    }

    // Communication
    item = cJSON_GetObjectItem(json, "uplink_mode");
    if (item && cJSON_IsNumber(item)) {
        new_cfg.uplink_mode = (nvs_uplink_mode_t)item->valueint;
    }

    item = cJSON_GetObjectItem(json, "mqtt_keepalive");
    if (item && cJSON_IsNumber(item)) {
        new_cfg.mqtt_keepalive = (uint16_t)item->valueint;
    }

    item = cJSON_GetObjectItem(json, "livedata_interval_ms");
    if (item && cJSON_IsNumber(item)) {
        new_cfg.livedata_interval_ms = (uint32_t)item->valueint;
    }

    item = cJSON_GetObjectItem(json, "softap_ssid_prefix");
    if (item && cJSON_IsString(item)) {
        strncpy(new_cfg.softap_ssid_prefix, item->valuestring, sizeof(new_cfg.softap_ssid_prefix) - 1);
        new_cfg.softap_ssid_prefix[sizeof(new_cfg.softap_ssid_prefix) - 1] = '\0';
    }

    // Mark as provisioned
    new_cfg.provisioned = true;

    cJSON_Delete(json);

    // Validate critical fields before saving
    {
        cJSON *err_resp = NULL;
        const char *err_msg = NULL;

        if (new_cfg.active_threshold <= 0.0f) {
            err_msg = "active_threshold must be > 0";
        } else {
            for (int i = 0; i < 3; i++) {
                if (new_cfg.ct_gain[i] <= 0.0f) {
                    err_msg = "ct_gain values must be > 0";
                    break;
                }
            }
        }
        if (!err_msg && (new_cfg.ct_enabled_mask > 0x07)) {
            err_msg = "ct_enabled_mask must be 0-7";
        }
        if (!err_msg && new_cfg.state_ct_phase_mode != NVS_CT_PHASE_ONE &&
            new_cfg.state_ct_phase_mode != NVS_CT_PHASE_THREE) {
            err_msg = "state_ct_phase_mode must be 0 or 1";
        }
        if (!err_msg && new_cfg.energy_ct_phase_mode != NVS_CT_PHASE_ONE &&
            new_cfg.energy_ct_phase_mode != NVS_CT_PHASE_THREE) {
            err_msg = "energy_ct_phase_mode must be 0 or 1";
        }
        if (!err_msg && new_cfg.state_source != NVS_STATE_SRC_CT &&
            new_cfg.state_source != NVS_STATE_SRC_SIGNAL) {
            err_msg = "state_source must be 0 or 1";
        }
        if (!err_msg && new_cfg.signal_logic != NVS_SIGNAL_LOGIC_NO &&
            new_cfg.signal_logic != NVS_SIGNAL_LOGIC_NC) {
            err_msg = "signal_logic must be 0 or 1";
        }
        if (!err_msg && (new_cfg.state_idle_stabilization_ms < 1000 ||
                         new_cfg.state_idle_stabilization_ms > 3600000)) {
            err_msg = "state_idle_stabilization_ms must be 1000-3600000";
        }
        if (!err_msg && (new_cfg.hysteresis_factor < 0.1f || new_cfg.hysteresis_factor >= 1.0f)) {
            err_msg = "hysteresis_factor must be in [0.1, 1.0)";
        }
        if (!err_msg && (new_cfg.active_sustain_ms < 100 || new_cfg.active_sustain_ms > 60000)) {
            err_msg = "active_sustain_ms must be 100-60000";
        }
        if (!err_msg && (new_cfg.machine_setup_time_ms < 100 || new_cfg.machine_setup_time_ms > 60000)) {
            err_msg = "machine_setup_time_ms must be 100-60000";
        }
        if (!err_msg && new_cfg.automize_ct_part_enabled &&
            new_cfg.automize_ct_part_threshold_a <= 0.0f) {
            err_msg = "CT count threshold must be > 0";
        }
        if (!err_msg && new_cfg.automize_ct_part_enabled &&
            (new_cfg.automize_ct_part_stabilization_ms == 0 ||
             new_cfg.automize_ct_part_stabilization_ms > 60000)) {
            err_msg = "CT count stabilization must be 1-60000 ms";
        }

        if (err_msg) {
            ESP_LOGE(TAG, "Config validation failed: %s", err_msg);
            err_resp = cJSON_CreateObject();
            cJSON_AddStringToObject(err_resp, "error", err_msg);
            httpd_resp_set_status(req, "400 Bad Request");
            send_json_response(req, err_resp);
            cJSON_Delete(err_resp);
            return ESP_OK;
        }
    }

    // Save to NVS
    esp_err_t ret = nvs_config_save(&new_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save config: %s", esp_err_to_name(ret));
        cJSON *error = cJSON_CreateObject();
        cJSON_AddStringToObject(error, "error", "Failed to save configuration");
        send_json_response(req, error);
        cJSON_Delete(error);
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Configuration saved successfully");

    // Send success response
    cJSON *response = cJSON_CreateObject();
    cJSON_AddBoolToObject(response, "success", true);
    cJSON_AddStringToObject(response, "message", "Configuration saved. Device will reboot.");
    send_json_response(req, response);
    cJSON_Delete(response);

    // Notify callback
    if (s_config_saved_cb) {
        s_config_saved_cb();
    }

    return ESP_OK;
}

/**
 * @brief Handler for POST /reboot - Trigger device reboot
 */
static esp_err_t reboot_post_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "POST /reboot - Reboot requested");

    cJSON *response = cJSON_CreateObject();
    cJSON_AddBoolToObject(response, "success", true);
    cJSON_AddStringToObject(response, "message", "Rebooting...");
    send_json_response(req, response);
    cJSON_Delete(response);

    // Notify callback to trigger reboot
    if (s_config_saved_cb) {
        s_config_saved_cb();
    }

    return ESP_OK;
}

/**
 * @brief Handler for GET /status - Get device status
 */
static esp_err_t status_get_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "GET /status");

    cJSON *response = cJSON_CreateObject();

    // Get heap info
    cJSON_AddNumberToObject(response, "free_heap", esp_get_free_heap_size());
    cJSON_AddNumberToObject(response, "min_free_heap", esp_get_minimum_free_heap_size());

    // Uptime
    int64_t uptime_us = esp_timer_get_time();
    cJSON_AddNumberToObject(response, "uptime_seconds", (double)(uptime_us / 1000000));

    // WiFi clients (in AP mode)
    wifi_sta_list_t sta_list;
    if (esp_wifi_ap_get_sta_list(&sta_list) == ESP_OK) {
        cJSON_AddNumberToObject(response, "connected_clients", sta_list.num);
    }

    send_json_response(req, response);
    cJSON_Delete(response);

    return ESP_OK;
}

/**
 * @brief Catch-all handler for captive portal
 */
static esp_err_t captive_portal_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "Captive portal redirect: %s", req->uri);
    httpd_resp_set_status(req, "302 Found");
    httpd_resp_set_hdr(req, "Location", "http://192.168.4.1/");
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
}

esp_err_t setup_webserver_start(webserver_config_saved_cb_t on_config_saved)
{
    if (s_active) {
        ESP_LOGW(TAG, "Webserver already running");
        return ESP_OK;
    }

    s_config_saved_cb = on_config_saved;

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_uri_handlers = 16;  // Increased for Windows captive portal endpoints
    config.uri_match_fn = httpd_uri_match_wildcard;
    config.stack_size = 8192;

    esp_err_t ret = httpd_start(&s_server, &config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start HTTP server: %s", esp_err_to_name(ret));
        return ret;
    }

    // Register URI handlers
    httpd_uri_t index_uri = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = index_get_handler,
    };
    httpd_register_uri_handler(s_server, &index_uri);

    httpd_uri_t generate_204_uri = {
        .uri = "/generate_204",
        .method = HTTP_GET,
        .handler = generate_204_handler,
    };
    httpd_register_uri_handler(s_server, &generate_204_uri);

    httpd_uri_t hotspot_uri = {
        .uri = "/hotspot-detect.html",
        .method = HTTP_GET,
        .handler = hotspot_detect_handler,
    };
    httpd_register_uri_handler(s_server, &hotspot_uri);

    httpd_uri_t scan_uri = {
        .uri = "/scan",
        .method = HTTP_GET,
        .handler = scan_get_handler,
    };
    httpd_register_uri_handler(s_server, &scan_uri);

    httpd_uri_t config_get_uri = {
        .uri = "/config",
        .method = HTTP_GET,
        .handler = config_get_handler,
    };
    httpd_register_uri_handler(s_server, &config_get_uri);

    httpd_uri_t config_post_uri = {
        .uri = "/config",
        .method = HTTP_POST,
        .handler = config_post_handler,
    };
    httpd_register_uri_handler(s_server, &config_post_uri);

    httpd_uri_t reboot_uri = {
        .uri = "/reboot",
        .method = HTTP_POST,
        .handler = reboot_post_handler,
    };
    httpd_register_uri_handler(s_server, &reboot_uri);

    httpd_uri_t status_uri = {
        .uri = "/status",
        .method = HTTP_GET,
        .handler = status_get_handler,
    };
    httpd_register_uri_handler(s_server, &status_uri);

    // Windows captive portal endpoints (must be before wildcard)
    httpd_uri_t connecttest_uri = {
        .uri = "/connecttest.txt",
        .method = HTTP_GET,
        .handler = windows_captive_handler,
    };
    httpd_register_uri_handler(s_server, &connecttest_uri);

    httpd_uri_t ncsi_uri = {
        .uri = "/ncsi.txt",
        .method = HTTP_GET,
        .handler = windows_captive_handler,
    };
    httpd_register_uri_handler(s_server, &ncsi_uri);

    httpd_uri_t redirect_uri = {
        .uri = "/redirect",
        .method = HTTP_GET,
        .handler = windows_captive_handler,
    };
    httpd_register_uri_handler(s_server, &redirect_uri);

    httpd_uri_t fwlink_uri = {
        .uri = "/fwlink",
        .method = HTTP_GET,
        .handler = windows_captive_handler,
    };
    httpd_register_uri_handler(s_server, &fwlink_uri);

    // Catch-all for captive portal (must be last)
    httpd_uri_t captive_uri = {
        .uri = "/*",
        .method = HTTP_GET,
        .handler = captive_portal_handler,
    };
    httpd_register_uri_handler(s_server, &captive_uri);

    s_active = true;
    ESP_LOGI(TAG, "HTTP server started on port 80");

    return ESP_OK;
}

esp_err_t setup_webserver_stop(void)
{
    if (!s_active || s_server == NULL) {
        return ESP_OK;
    }

    esp_err_t ret = httpd_stop(s_server);
    if (ret == ESP_OK) {
        s_server = NULL;
        s_active = false;
        ESP_LOGI(TAG, "HTTP server stopped");
    } else {
        ESP_LOGE(TAG, "Failed to stop HTTP server: %s", esp_err_to_name(ret));
    }

    return ret;
}

bool setup_webserver_is_active(void)
{
    return s_active;
}
