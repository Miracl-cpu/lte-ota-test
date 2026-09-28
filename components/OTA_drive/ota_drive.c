/**
 * @file ota_drive.c
 * @brief OTAdrive OTA update module implementation
 *
 * Uses existing WiFi connection from wifi_manager.
 * Communicates with OTAdrive cloud to check/download firmware updates.
 */

#include "ota_drive.h"
#include "ota_progress_tracker.h"

#include "esp_event.h"
#include "esp_app_desc.h"
#include "esp_https_ota.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "log_config.h"
#undef LOG_LOCAL_LEVEL
#define LOG_LOCAL_LEVEL LOG_LEVEL_OTA
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "nvs.h"
#include "wifi_manager.h"

#include <otadrive_esp.h>
#include <ctype.h>
#include <stdio.h>
#include <string.h>
#include <psa/crypto.h>

// ============================================================================
// Private definitions
// ============================================================================

static const char *TAG = "ota_drive";

// ============================================================================
// Private state
// ============================================================================

static bool s_initialized = false;
static volatile bool s_ota_in_progress = false;
static SemaphoreHandle_t s_ota_mutex = NULL;
static char s_runtime_version[sizeof(((esp_app_desc_t *)0)->version) + 3];

#define OTA_MARKER_NAMESPACE "ota_meta"
#define OTA_MARKER_KEY       "updated"
#define PRIVATE_OTA_MARKER_NAMESPACE "private_ota"
#define PRIVATE_OTA_MARKER_KEY       "boot_success"
#define PRIVATE_OTA_DEPLOYMENT_KEY   "deployment_id"
#define PRIVATE_OTA_COMMAND_KEY      "command_id"
#define PRIVATE_OTA_VERSION_KEY      "version"

/* HTTP is restricted to the current lab server. Production URLs must use
 * HTTPS; CONFIG_ESP_HTTPS_OTA_ALLOW_HTTP must be disabled in production. */
#define PRIVATE_OTA_LAB_HTTP_PREFIX "http://94.136.187.195:9000/"
#define PRIVATE_OTA_HTTP_TIMEOUT_MS 10000

static esp_err_t save_update_boot_marker(void) {
    nvs_handle_t handle;
    esp_err_t ret = nvs_open(OTA_MARKER_NAMESPACE, NVS_READWRITE, &handle);
    if (ret != ESP_OK) return ret;

    ret = nvs_set_u8(handle, OTA_MARKER_KEY, 1);
    if (ret == ESP_OK) ret = nvs_commit(handle);
    nvs_close(handle);
    return ret;
}

static bool private_ota_url_allowed(const char *url) {
    if (url == NULL) return false;
    return strncmp(url, "https://", 8) == 0 ||
           strncmp(url, PRIVATE_OTA_LAB_HTTP_PREFIX,
                   strlen(PRIVATE_OTA_LAB_HTTP_PREFIX)) == 0;
}

static bool private_ota_hex_valid(const char *value) {
    if (value == NULL || strlen(value) != PRIVATE_OTA_SHA256_HEX_LEN) return false;
    for (size_t i = 0; i < PRIVATE_OTA_SHA256_HEX_LEN; ++i) {
        if (!isxdigit((unsigned char)value[i])) return false;
    }
    return true;
}

static void private_ota_emit(private_ota_status_cb_t cb,
                             const private_ota_request_t *request,
                             const char *status, int progress,
                             const char *error, void *context) {
    if (cb != NULL) cb(request, status, progress, error, context);
}

static esp_err_t private_ota_save_boot_success(const private_ota_request_t *request) {
    nvs_handle_t handle;
    esp_err_t ret = nvs_open(PRIVATE_OTA_MARKER_NAMESPACE, NVS_READWRITE, &handle);
    if (ret != ESP_OK) return ret;
    ret = nvs_set_u8(handle, PRIVATE_OTA_MARKER_KEY, 1);
    if (ret == ESP_OK) ret = nvs_set_str(handle, PRIVATE_OTA_DEPLOYMENT_KEY, request->deployment_id);
    if (ret == ESP_OK) ret = nvs_set_str(handle, PRIVATE_OTA_COMMAND_KEY, request->command_id);
    if (ret == ESP_OK) ret = nvs_set_str(handle, PRIVATE_OTA_VERSION_KEY, request->version);
    if (ret == ESP_OK) ret = nvs_commit(handle);
    nvs_close(handle);
    return ret;
}

static void private_ota_hex(const uint8_t *source, size_t len, char *dest,
                            size_t dest_len) {
    static const char digits[] = "0123456789abcdef";
    if (dest_len < (len * 2U) + 1U) return;
    for (size_t i = 0; i < len; ++i) {
        dest[i * 2U] = digits[source[i] >> 4U];
        dest[i * 2U + 1U] = digits[source[i] & 0x0FU];
    }
    dest[len * 2U] = '\0';
}

/* Hash the exact .bin byte stream stored in the OTA partition. This includes
 * the ESP image's appended digest and therefore matches a normal SHA-256 of
 * the complete firmware artifact supplied by the deployment service. */
static esp_err_t private_ota_partition_file_sha256(const esp_partition_t *partition,
                                                   size_t image_size,
                                                   char *sha256_hex,
                                                   size_t sha256_hex_len) {
    enum { HASH_READ_CHUNK_SIZE = 1024 };
    uint8_t chunk[HASH_READ_CHUNK_SIZE];
    uint8_t digest[32];
    size_t digest_len = 0;
    psa_hash_operation_t operation = PSA_HASH_OPERATION_INIT;

    psa_status_t psa_ret = psa_crypto_init();
    if (psa_ret != PSA_SUCCESS) {
        ESP_LOGE(TAG, "PSA crypto init failed: %ld", (long)psa_ret);
        return ESP_FAIL;
    }
    psa_ret = psa_hash_setup(&operation, PSA_ALG_SHA_256);
    if (psa_ret != PSA_SUCCESS) {
        ESP_LOGE(TAG, "SHA-256 setup failed: %ld", (long)psa_ret);
        return ESP_FAIL;
    }

    esp_err_t ret = ESP_OK;
    for (size_t offset = 0; offset < image_size;) {
        size_t chunk_len = image_size - offset;
        if (chunk_len > sizeof(chunk)) chunk_len = sizeof(chunk);

        ret = esp_partition_read(partition, offset, chunk, chunk_len);
        if (ret != ESP_OK) break;

        psa_ret = psa_hash_update(&operation, chunk, chunk_len);
        if (psa_ret != PSA_SUCCESS) {
            ESP_LOGE(TAG, "SHA-256 update failed: %ld", (long)psa_ret);
            ret = ESP_FAIL;
            break;
        }
        offset += chunk_len;
    }

    if (ret == ESP_OK) {
        psa_ret = psa_hash_finish(&operation, digest, sizeof(digest), &digest_len);
        if (psa_ret != PSA_SUCCESS || digest_len != sizeof(digest)) {
            ESP_LOGE(TAG, "SHA-256 finish failed: status=%ld length=%u",
                     (long)psa_ret, (unsigned)digest_len);
            ret = ESP_FAIL;
        }
    }
    if (ret != ESP_OK) {
        (void)psa_hash_abort(&operation);
        return ret;
    }

    private_ota_hex(digest, sizeof(digest), sha256_hex, sha256_hex_len);
    return ESP_OK;
}

// ============================================================================
// OTA Event Handler (progress tracking)
// ============================================================================

static ota_progress_cb_t s_progress_callback = NULL;

static void ota_event_handler(void *arg, esp_event_base_t event_base,
                              int32_t event_id, void *event_data) {
    if (event_base != ESP_HTTPS_OTA_EVENT) return;

    const char *status_msg = "";
    int progress = -1;

    switch (event_id) {
        case ESP_HTTPS_OTA_START:
            status_msg = "OTA started";
            progress = 0;
            break;
        case ESP_HTTPS_OTA_CONNECTED:
            status_msg = "Connected to OTA server";
            progress = 5;
            break;
        case ESP_HTTPS_OTA_GET_IMG_DESC:
            status_msg = "Reading image description";
            progress = 10;
            break;
        case ESP_HTTPS_OTA_VERIFY_CHIP_ID:
            status_msg = "Verifying chip ID";
            progress = 15;
            break;
        case ESP_HTTPS_OTA_WRITE_FLASH:
            if (event_data) {
                int written = *(int *)event_data;
                progress = 20 + (written * 70 / (1024 * 1024));
                if (progress > 90) progress = 90;
            }
            status_msg = "Writing to flash";
            break;
        case ESP_HTTPS_OTA_UPDATE_BOOT_PARTITION:
            status_msg = "Updating boot partition";
            progress = 95;
            break;
        case ESP_HTTPS_OTA_FINISH:
            status_msg = "OTA complete";
            progress = 100;
            break;
        case ESP_HTTPS_OTA_ABORT:
            status_msg = "OTA aborted";
            progress = -1;
            break;
        default:
            return;
    }

    ESP_LOGI(TAG, "[OTA] %s (progress: %d%%)", status_msg, progress);

    if (s_progress_callback && progress >= 0) {
        s_progress_callback(progress, status_msg);
    }
}

// ============================================================================
// Public API Implementation
// ============================================================================

esp_err_t ota_drive_init(void) {
    if (s_initialized) {
        ESP_LOGW(TAG, "Already initialized");
        return ESP_OK;
    }

    // Create mutex for thread safety
    s_ota_mutex = xSemaphoreCreateMutex();
    if (!s_ota_mutex) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return ESP_FAIL;
    }

    // Register OTA progress event handler
    esp_err_t ret = esp_event_handler_register(
        ESP_HTTPS_OTA_EVENT, ESP_EVENT_ANY_ID, &ota_event_handler, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register OTA event handler: %s",
                 esp_err_to_name(ret));
        return ret;
    }

    // Initialize OTAdrive with API key and current version
    char current_version[sizeof(s_runtime_version)];
    snprintf(current_version, sizeof(current_version), "%s", ota_drive_get_version());
    otadrive_setInfo(OTA_DRIVE_APIKEY, current_version);

    s_initialized = true;
    ESP_LOGI(TAG, "OTA Drive initialized (version: %s)", ota_drive_get_version());

    return ESP_OK;
}

ota_status_t ota_drive_check_and_update(ota_progress_cb_t progress_cb) {
    if (!s_initialized) {
        ESP_LOGE(TAG, "Module not initialized, call ota_drive_init() first");
        return OTA_STATUS_NOT_INITIALIZED;
    }

    // Try to acquire mutex (non-blocking check)
    if (xSemaphoreTake(s_ota_mutex, 0) != pdTRUE) {
        ESP_LOGW(TAG, "OTA already in progress");
        return OTA_STATUS_ALREADY_IN_PROGRESS;
    }

    s_ota_in_progress = true;
    s_progress_callback = progress_cb;
    ota_status_t status = OTA_STATUS_OK;

    // Check WiFi connectivity
    if (!wifi_manager_is_connected()) {
        ESP_LOGE(TAG, "WiFi not connected — cannot check for OTA");
        status = OTA_STATUS_NO_WIFI;
        goto cleanup;
    }

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "Starting OTA check...");
    ESP_LOGI(TAG, "Current version: %s", VERN);
    ESP_LOGI(TAG, "========================================");

    // Step 1: Check for updates
    if (progress_cb) progress_cb(10, "Checking for updates...");

    ESP_LOGI(TAG, "Querying OTAdrive server...");
    otadrive_result result = otadrive_updateFirmwareInfo();

    ESP_LOGI(TAG, "OTA check result: code=%d, available=%d, size=%lu",
             result.code, result.available, result.size);

    if (!result.available) {
        ESP_LOGI(TAG, "No update available. Firmware is up to date.");
        status = OTA_STATUS_NO_UPDATE_AVAILABLE;
        goto cleanup;
    }

    // Step 2: Download and flash
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "New firmware available!");
    ESP_LOGI(TAG, "  New version: %s", result.version);
    ESP_LOGI(TAG, "  Size: %lu bytes", result.size);
    ESP_LOGI(TAG, "  Current: %s", otadrive_currentversion());
    ESP_LOGI(TAG, "========================================");

    if (progress_cb) progress_cb(15, "Downloading firmware...");

    ESP_LOGI(TAG, "Starting firmware download and flash...");
    result = otadrive_updateFirmware(false);

    if (result.code == OTADRIVE_Success) {
        ESP_LOGI(TAG, "========================================");
        ESP_LOGI(TAG, "OTA UPDATE SUCCESSFUL!");
        ESP_LOGI(TAG, "Rebooting in 3 seconds...");
        ESP_LOGI(TAG, "========================================");

        if (progress_cb) progress_cb(100, "Update complete! Rebooting...");

        /* Persist a one-shot marker so the new image can distinguish this
         * restart from an ordinary software-requested reboot. */
        esp_err_t marker_ret = save_update_boot_marker();
        if (marker_ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to save OTA boot marker: %s",
                     esp_err_to_name(marker_ret));
        }

        vTaskDelay(pdMS_TO_TICKS(3000));
        esp_restart();

        // Won't reach here
        status = OTA_STATUS_UPDATE_SUCCESS;
    } else {
        ESP_LOGE(TAG, "OTA download/flash failed with code: %d", result.code);
        status = OTA_STATUS_DOWNLOAD_FAILED;
    }

cleanup:
    s_ota_in_progress = false;
    s_progress_callback = NULL;
    xSemaphoreGive(s_ota_mutex);

    return status;
}

esp_err_t ota_private_update(const private_ota_request_t *request,
                             private_ota_status_cb_t status_cb,
                             void *context) {
    if (request == NULL || request->command_id[0] == '\0' ||
        request->deployment_id[0] == '\0' || request->version[0] == '\0' ||
        request->url[0] == '\0' || request->size == 0 ||
        !private_ota_hex_valid(request->sha256)) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!wifi_manager_is_connected()) {
        private_ota_emit(status_cb, request, "FAILED", 0, "wifi_not_connected", context);
        return ESP_ERR_INVALID_STATE;
    }
    if (!private_ota_url_allowed(request->url)) {
        private_ota_emit(status_cb, request, "FAILED", 0, "url_not_allowed", context);
        return ESP_ERR_INVALID_ARG;
    }
    if (ota_drive_is_busy()) {
        private_ota_emit(status_cb, request, "FAILED", 0, "ota_busy", context);
        return ESP_ERR_INVALID_STATE;
    }

    const esp_partition_t *target = esp_ota_get_next_update_partition(NULL);
    if (target == NULL || request->size > target->size) {
        private_ota_emit(status_cb, request, "FAILED", 0, "image_too_large", context);
        return ESP_ERR_INVALID_SIZE;
    }

    esp_http_client_config_t http_config = {
        .url = request->url,
        .timeout_ms = PRIVATE_OTA_HTTP_TIMEOUT_MS,
        .keep_alive_enable = true,
    };
    esp_https_ota_config_t ota_config = {
        .http_config = &http_config,
        .partial_http_download = false,
    };
    esp_https_ota_handle_t handle = NULL;
    esp_err_t ret;
    ota_progress_tracker_t progress = ota_progress_tracker_init(request->size, esp_timer_get_time());

    s_ota_in_progress = true;
    private_ota_emit(status_cb, request, "DOWNLOADING", 0, NULL, context);
    ret = esp_https_ota_begin(&ota_config, &handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Private OTA begin failed: %s", esp_err_to_name(ret));
        private_ota_emit(status_cb, request, "FAILED", 0, "download_start_failed", context);
        goto cleanup;
    }
    int response_size = esp_https_ota_get_image_size(handle);
    if (response_size < 0 || (size_t)response_size != request->size) {
        ESP_LOGE(TAG, "Private OTA Content-Length mismatch: response=%d expected=%u",
                 response_size, (unsigned)request->size);
        private_ota_emit(status_cb, request, "FAILED", 0, "size_mismatch", context);
        ret = ESP_ERR_INVALID_SIZE;
        goto cleanup;
    }

    ESP_LOGI(TAG, "Private OTA download started: expected=%u bytes", (unsigned)request->size);
    for (;;) {
        const char *timeout_error = ota_progress_tracker_timeout(&progress, esp_timer_get_time());
        if (timeout_error != NULL) {
            ESP_LOGE(TAG, "Private OTA %s: received=%u expected=%u", timeout_error,
                     (unsigned)progress.received_bytes, (unsigned)request->size);
            private_ota_emit(status_cb, request, "FAILED", ota_progress_tracker_percent(&progress),
                             timeout_error, context);
            ret = ESP_ERR_TIMEOUT;
            goto cleanup;
        }
        if (!wifi_manager_is_connected()) {
            private_ota_emit(status_cb, request, "FAILED", ota_progress_tracker_percent(&progress),
                             "wifi_disconnected", context);
            ret = ESP_ERR_INVALID_STATE;
            goto cleanup;
        }

        ret = esp_https_ota_perform(handle);
        if (ret != ESP_OK && ret != ESP_ERR_HTTPS_OTA_IN_PROGRESS) break;

        int received = esp_https_ota_get_image_len_read(handle);
        if (received < 0 || (size_t)received > request->size) {
            ESP_LOGE(TAG, "Private OTA invalid byte count: received=%d expected=%u",
                     received, (unsigned)request->size);
            private_ota_emit(status_cb, request, "FAILED", ota_progress_tracker_percent(&progress),
                             "size_mismatch", context);
            ret = ESP_ERR_INVALID_SIZE;
            goto cleanup;
        }
        int64_t now_us = esp_timer_get_time();
        ota_progress_tracker_observe(&progress, (size_t)received, now_us);
        timeout_error = ota_progress_tracker_timeout(&progress, now_us);
        if (timeout_error != NULL) {
            ESP_LOGE(TAG, "Private OTA %s: received=%d expected=%u", timeout_error,
                     received, (unsigned)request->size);
            private_ota_emit(status_cb, request, "FAILED", ota_progress_tracker_percent(&progress),
                             timeout_error, context);
            ret = ESP_ERR_TIMEOUT;
            goto cleanup;
        }
        if (ota_progress_tracker_should_report(&progress, now_us)) {
            int percent = ota_progress_tracker_percent(&progress);
            ESP_LOGI(TAG, "Private OTA download: %d%% (%d/%u bytes)",
                     percent, received, (unsigned)request->size);
            private_ota_emit(status_cb, request, "DOWNLOADING", percent, NULL, context);
        }
        if (ret == ESP_OK) break;
        /* Yield to WiFi/MQTT and the idle task between chunks/retries. */
        vTaskDelay(1);
    }
    if (ret != ESP_OK || !esp_https_ota_is_complete_data_received(handle)) {
        if (ret == ESP_OK) ret = ESP_FAIL;
        ESP_LOGE(TAG, "Private OTA download failed: %s (%u/%u bytes)", esp_err_to_name(ret),
                 (unsigned)progress.received_bytes, (unsigned)request->size);
        private_ota_emit(status_cb, request, "FAILED", ota_progress_tracker_percent(&progress),
                         "download_failed", context);
        goto cleanup;
    }
    if ((size_t)esp_https_ota_get_image_len_read(handle) != request->size) {
        ESP_LOGE(TAG, "Private OTA size mismatch: received=%d expected=%u",
                 esp_https_ota_get_image_len_read(handle), (unsigned)request->size);
        private_ota_emit(status_cb, request, "FAILED", 0, "size_mismatch", context);
        ret = ESP_ERR_INVALID_SIZE;
        goto cleanup;
    }

    private_ota_emit(status_cb, request, "VERIFYING", 100, NULL, context);
    esp_app_desc_t app_desc;
    ret = esp_ota_get_partition_description(target, &app_desc);
    if (ret != ESP_OK || strcmp(app_desc.version, request->version) != 0) {
        ESP_LOGE(TAG, "Private OTA version mismatch: image=%s requested=%s",
                 ret == ESP_OK ? app_desc.version : "unreadable", request->version);
        private_ota_emit(status_cb, request, "FAILED", 0, "version_mismatch", context);
        if (ret == ESP_OK) ret = ESP_ERR_INVALID_VERSION;
        goto cleanup;
    }

    char artifact_sha_hex[PRIVATE_OTA_SHA256_HEX_LEN + 1];
    ret = private_ota_partition_file_sha256(target, request->size,
                                            artifact_sha_hex, sizeof(artifact_sha_hex));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to hash downloaded firmware artifact: %s", esp_err_to_name(ret));
        private_ota_emit(status_cb, request, "FAILED", 100, "sha256_read_failed", context);
        goto cleanup;
    }
    if (strcasecmp(artifact_sha_hex, request->sha256) != 0) {
        ESP_LOGE(TAG, "Downloaded firmware artifact SHA-256 does not match request");
        private_ota_emit(status_cb, request, "FAILED", 100, "sha256_mismatch", context);
        ret = ESP_ERR_INVALID_CRC;
        goto cleanup;
    }

    /* Keep ESP-IDF's own image-digest verification as an independent check. */
    uint8_t image_sha[32];
    ret = esp_partition_get_sha256(target, image_sha);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ESP application image integrity check failed: %s", esp_err_to_name(ret));
        private_ota_emit(status_cb, request, "FAILED", 100, "image_integrity_failed", context);
        goto cleanup;
    }

    ret = esp_https_ota_finish(handle);
    handle = NULL;
    if (ret != ESP_OK) {
        private_ota_emit(status_cb, request, "FAILED", 0, "install_failed", context);
        goto cleanup;
    }
    ret = private_ota_save_boot_success(request);
    if (ret != ESP_OK) {
        private_ota_emit(status_cb, request, "FAILED", 0, "boot_marker_failed", context);
        goto cleanup;
    }

    private_ota_emit(status_cb, request, "INSTALLING", 100, NULL, context);
    ESP_LOGI(TAG, "Private OTA validated; restarting into version %s", request->version);
    vTaskDelay(pdMS_TO_TICKS(1000));
    esp_restart();

cleanup:
    if (handle != NULL) esp_https_ota_abort(handle);
    s_ota_in_progress = false;
    return ret;
}

const char *ota_drive_get_version(void) {
    const esp_app_desc_t *description = esp_app_get_description();
    if (description != NULL && description->version[0] != '\0') {
        int written = snprintf(s_runtime_version, sizeof(s_runtime_version),
                               "v@%s", description->version);
        if (written > 0 && written < (int)sizeof(s_runtime_version)) {
            return s_runtime_version;
        }
    }
    return VERN;
}

bool ota_drive_is_busy(void) {
    return s_ota_in_progress;
}

bool ota_private_take_boot_success(private_ota_request_t *out) {
    if (out == NULL) return false;
    memset(out, 0, sizeof(*out));
    nvs_handle_t handle;
    if (nvs_open(PRIVATE_OTA_MARKER_NAMESPACE, NVS_READWRITE, &handle) != ESP_OK) return false;
    uint8_t marker = 0;
    bool present = nvs_get_u8(handle, PRIVATE_OTA_MARKER_KEY, &marker) == ESP_OK && marker == 1;
    if (present) {
        size_t length = sizeof(out->deployment_id);
        esp_err_t ret = nvs_get_str(handle, PRIVATE_OTA_DEPLOYMENT_KEY, out->deployment_id, &length);
        length = sizeof(out->command_id);
        if (ret == ESP_OK) ret = nvs_get_str(handle, PRIVATE_OTA_COMMAND_KEY, out->command_id, &length);
        length = sizeof(out->version);
        if (ret == ESP_OK) ret = nvs_get_str(handle, PRIVATE_OTA_VERSION_KEY, out->version, &length);
        if (ret == ESP_OK) {
            nvs_erase_key(handle, PRIVATE_OTA_MARKER_KEY);
            nvs_erase_key(handle, PRIVATE_OTA_DEPLOYMENT_KEY);
            nvs_erase_key(handle, PRIVATE_OTA_COMMAND_KEY);
            nvs_erase_key(handle, PRIVATE_OTA_VERSION_KEY);
            ret = nvs_commit(handle);
        }
        present = ret == ESP_OK;
    }
    nvs_close(handle);
    return present;
}

bool ota_drive_consume_update_boot_marker(void) {
    nvs_handle_t handle;
    esp_err_t ret = nvs_open(OTA_MARKER_NAMESPACE, NVS_READWRITE, &handle);
    if (ret != ESP_OK) return false;

    uint8_t marker = 0;
    bool was_updated =
        nvs_get_u8(handle, OTA_MARKER_KEY, &marker) == ESP_OK && marker == 1;
    if (was_updated) {
        ret = nvs_erase_key(handle, OTA_MARKER_KEY);
        if (ret == ESP_OK) ret = nvs_commit(handle);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to consume OTA boot marker: %s",
                     esp_err_to_name(ret));
            was_updated = false;
        }
    }
    nvs_close(handle);
    return was_updated;
}
