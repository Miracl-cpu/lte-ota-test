/**
 * @file ota_drive.h
 * @brief OTAdrive OTA update module for Edge IoT Gateway
 *
 * Performs OTA firmware updates via OTAdrive cloud service.
 * Requires an existing WiFi connection (managed by wifi_manager).
 *
 * Usage:
 *   1. Call ota_drive_init() once at startup (after WiFi is initialized)
 *   2. Call ota_drive_check_and_update() when triggered (e.g., via MQTT command)
 */

#ifndef OTA_DRIVE_H
#define OTA_DRIVE_H

#include "esp_err.h"
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// Configuration - Modify these for your deployment
// ============================================================================

// OTAdrive API keys — one product per hardware variant (app.otadrive.com)
#define OTA_APIKEY_4MB   "e1e4680c-ac67-4fce-b429-c754e18d41f1"
#define OTA_APIKEY_8MB   "d93a6412-df2e-4150-9c17-3c1d9f475553"
#define OTA_APIKEY_16MB  "0bda416c-b70b-48a4-9b5f-f9be1ad54669"

// Auto-selected at compile time by FLASH_VARIANT set in CMakeLists.txt.
// Change variant: edit idf.cmakeExtraArgs in .vscode/settings.json.
#if defined(FLASH_VARIANT_4MB)
  #define OTA_DRIVE_APIKEY  OTA_APIKEY_4MB
#elif defined(FLASH_VARIANT_8MB)
  #define OTA_DRIVE_APIKEY  OTA_APIKEY_8MB
#else
  #define OTA_DRIVE_APIKEY  OTA_APIKEY_16MB
#endif

#define VERN                    "v@3.1.7"

// ============================================================================
// Status codes
// ============================================================================

typedef enum {
    OTA_STATUS_OK = 0,
    OTA_STATUS_NO_WIFI,                // WiFi not connected
    OTA_STATUS_NO_UPDATE_AVAILABLE,
    OTA_STATUS_UPDATE_AVAILABLE,
    OTA_STATUS_DOWNLOAD_FAILED,
    OTA_STATUS_UPDATE_SUCCESS,         // Will reboot after this
    OTA_STATUS_ALREADY_IN_PROGRESS,
    OTA_STATUS_NOT_INITIALIZED,
} ota_status_t;

// ============================================================================
// Callback type for OTA progress (optional)
// ============================================================================

typedef void (*ota_progress_cb_t)(int percent_complete, const char *status_msg);

/* Dashboard OTA limits. Device targeting is enforced by the per-device
 * MQTT config topic, not by a firmware-side device allowlist. */
#define PRIVATE_OTA_MAX_URL_LEN 1024
#define PRIVATE_OTA_MAX_ID_LEN 64
#define PRIVATE_OTA_MAX_VERSION_LEN 32
#define PRIVATE_OTA_SHA256_HEX_LEN 64

typedef struct {
    char command_id[PRIVATE_OTA_MAX_ID_LEN];
    char deployment_id[PRIVATE_OTA_MAX_ID_LEN];
    char version[PRIVATE_OTA_MAX_VERSION_LEN];
    char url[PRIVATE_OTA_MAX_URL_LEN];
    char sha256[PRIVATE_OTA_SHA256_HEX_LEN + 1];
    size_t size;
} private_ota_request_t;

typedef void (*private_ota_status_cb_t)(const private_ota_request_t *request,
                                        const char *status,
                                        int progress_percent,
                                        const char *error_code,
                                        void *context);

// ============================================================================
// Public API
// ============================================================================

/**
 * @brief Initialize the OTA drive module
 * @note Call once at startup. Does NOT trigger an update.
 * @return ESP_OK on success
 */
esp_err_t ota_drive_init(void);

/**
 * @brief Check for OTA update and apply if available
 *
 * Requires WiFi to be already connected (via wifi_manager).
 * If an update is available, it will be downloaded, flashed,
 * and the device will reboot.
 *
 * @param progress_cb Optional callback for progress updates (can be NULL)
 * @return ota_status_t result code
 */
ota_status_t ota_drive_check_and_update(ota_progress_cb_t progress_cb);

/**
 * @brief Get current firmware version string
 * @return Version string
 */
const char *ota_drive_get_version(void);

/**
 * @brief Check if OTA is currently in progress
 * @return true if OTA operation is running
 */
bool ota_drive_is_busy(void);

/** Download a dashboard-supplied image to the inactive OTA partition.
 * This function is blocking and must only run in the dedicated OTA task. */
esp_err_t ota_private_update(const private_ota_request_t *request,
                             private_ota_status_cb_t status_cb,
                             void *context);

/** Save the deployment identity before reboot, then retrieve it after a
 * successful private OTA boot so the caller can publish SUCCESS. */
bool ota_private_take_boot_success(private_ota_request_t *out);

/**
 * @brief Consume the one-shot marker written after a successful OTA download
 * @return true only on the first boot following that OTA update
 */
bool ota_drive_consume_update_boot_marker(void);

#ifdef __cplusplus
}
#endif

#endif // OTA_DRIVE_H
