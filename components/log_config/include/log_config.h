/**
 * @file log_config.h
 * @brief Centralized serial log level control for all modules
 *
 * Usage:
 *   1. Set GLOBAL_LOG_ENABLE to 0 to disable ALL serial logs (production build)
 *   2. Set individual LOG_LEVEL_xxx macros to control per-module verbosity
 *
 * Log levels (from most to least verbose):
 *   ESP_LOG_VERBOSE  (5) - Detailed debug traces
 *   ESP_LOG_DEBUG    (4) - Debug messages
 *   ESP_LOG_INFO     (3) - General information
 *   ESP_LOG_WARN     (2) - Warnings only
 *   ESP_LOG_ERROR    (1) - Errors only
 *   ESP_LOG_NONE     (0) - Completely disabled (saves flash)
 *
 * Include this header BEFORE esp_log.h in each .c file:
 *   #include "log_config.h"
 *   #define LOG_LOCAL_LEVEL LOG_LEVEL_xxx
 *   #include "esp_log.h"
 */

#ifndef LOG_CONFIG_H
#define LOG_CONFIG_H

/*
 * Use esp_log_level.h (not esp_log.h) to get the ESP_LOG_* level constants.
 * esp_log.h defines LOG_LOCAL_LEVEL (to CONFIG_LOG_DEFAULT_LEVEL) if it is
 * not already set, which would cause a "redefined" warning in every .c file
 * that does:
 *   #include "log_config.h"
 *   #define LOG_LOCAL_LEVEL LOG_LEVEL_xxx   ← redefinition if esp_log.h already ran
 *   #include "esp_log.h"
 * esp_log_level.h provides the level enum/constants without touching
 * LOG_LOCAL_LEVEL, so each .c file can set it freely before esp_log.h.
 */
#include "esp_log_level.h"

// ============================================================================
// MASTER SWITCH — set to 0 to disable ALL serial logs (production builds)
// ============================================================================
#define GLOBAL_LOG_ENABLE       1

// ============================================================================
// PER-MODULE LOG LEVELS
// Change individual levels here to debug specific modules.
// When GLOBAL_LOG_ENABLE is 0, all modules are forced to ESP_LOG_NONE.
// ============================================================================

#if GLOBAL_LOG_ENABLE

#define LOG_LEVEL_MAIN              ESP_LOG_INFO      // app_main + PROV_DBG
#define LOG_LEVEL_MQTT              ESP_LOG_INFO      // mqtt_client_mgr
#define LOG_LEVEL_WIFI              ESP_LOG_INFO      // wifi_manager
#define LOG_LEVEL_OTA               ESP_LOG_INFO      // ota_drive
#define LOG_LEVEL_NVS               ESP_LOG_INFO      // nvs_config
#define LOG_LEVEL_DAQ_CT            ESP_LOG_VERBOSE   // daq_ct_ads1115
#define LOG_LEVEL_JSON_LOGGER       ESP_LOG_INFO      // json_data_logger
#define LOG_LEVEL_I2C_MUX           ESP_LOG_VERBOSE   // i2c_mux_mgr
#define LOG_LEVEL_SNTP              ESP_LOG_INFO      // sntp_time
#define LOG_LEVEL_KEYPAD            ESP_LOG_VERBOSE   // keypad_pcf8574_matrix
#define LOG_LEVEL_LCD               ESP_LOG_VERBOSE   // datameter_lcd
#define LOG_LEVEL_PROV_MGR          ESP_LOG_INFO      // provisioning_mgr
#define LOG_LEVEL_SETUP_SOFTAP      ESP_LOG_INFO      // setup_softap
#define LOG_LEVEL_SETUP_WEBSERVER   ESP_LOG_INFO      // setup_webserver
#define LOG_LEVEL_SETUP_TRIGGER     ESP_LOG_INFO      // setup_trigger
#define LOG_LEVEL_CAVLI             ESP_LOG_DEBUG     // cavli_lte_mgr
#define LOG_LEVEL_VIB               ESP_LOG_INFO      // wtvb01_485

#else  // GLOBAL_LOG_ENABLE == 0 → silence everything

#define LOG_LEVEL_MAIN              ESP_LOG_NONE
#define LOG_LEVEL_MQTT              ESP_LOG_NONE
#define LOG_LEVEL_WIFI              ESP_LOG_NONE
#define LOG_LEVEL_OTA               ESP_LOG_NONE
#define LOG_LEVEL_NVS               ESP_LOG_NONE
#define LOG_LEVEL_DAQ_CT            ESP_LOG_NONE
#define LOG_LEVEL_JSON_LOGGER       ESP_LOG_NONE
#define LOG_LEVEL_I2C_MUX           ESP_LOG_NONE
#define LOG_LEVEL_SNTP              ESP_LOG_NONE
#define LOG_LEVEL_KEYPAD            ESP_LOG_NONE
#define LOG_LEVEL_LCD               ESP_LOG_NONE
#define LOG_LEVEL_PROV_MGR          ESP_LOG_NONE
#define LOG_LEVEL_SETUP_SOFTAP      ESP_LOG_NONE
#define LOG_LEVEL_SETUP_WEBSERVER   ESP_LOG_NONE
#define LOG_LEVEL_SETUP_TRIGGER     ESP_LOG_NONE
#define LOG_LEVEL_CAVLI             ESP_LOG_NONE
#define LOG_LEVEL_VIB               ESP_LOG_NONE

#endif // GLOBAL_LOG_ENABLE

#endif // LOG_CONFIG_H
