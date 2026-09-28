/**
 * @file i2c_mux_mgr.c
 * @brief I2C Multiplexer Manager implementation
 */

#include "i2c_mux_mgr.h"
#include "log_config.h"
#undef LOG_LOCAL_LEVEL
#define LOG_LOCAL_LEVEL LOG_LEVEL_I2C_MUX
#include "esp_log.h"
#include <string.h>

static const char *TAG = "i2c_mux_mgr";

// Mutex timeout for i2c_mux_mgr_exec().
// 200 ms is 40× the longest normal DAQ I2C transaction (~5 ms).
// Keeping this low bounds worst-case blocking time in hmi_task:
//   LCD/keypad operations should fail quickly instead of blocking DAQ sampling.
//   17 × (200ms mutex wait + 20ms scl_wait_us) = ~3.7 s — well under 10 s TWDT.
// With the old value of 5000 ms: 17 × 5020 ms = 85 s → guaranteed TWDT fire.
#define MUX_MUTEX_TIMEOUT_MS     200
// Number of consecutive i2c_mux_mgr_exec() failures before calling i2c_master_bus_reset().
// Set to 1: trigger recovery on the very first persistent failure.
// When scl_wait_us is set correctly each failed call returns in ~20ms, so recovery fires
// within ~20ms of a stuck bus instead of waiting 5 × 20ms = 100ms.
#define I2C_RECOVERY_THRESHOLD   20

esp_err_t i2c_mux_mgr_init(i2c_mux_mgr_t *mgr, gpio_num_t sda, gpio_num_t scl,
                           uint32_t freq_hz, uint8_t tca_addr)
{
    if (mgr == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(mgr, 0, sizeof(i2c_mux_mgr_t));
    mgr->tca_addr = tca_addr;
    mgr->cached_channel = -1;
    mgr->bus_recovering = false;

    // Create mutex
    mgr->mutex = xSemaphoreCreateMutex();
    if (mgr->mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return ESP_ERR_NO_MEM;
    }

    // Configure I2C master bus
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = sda,
        .scl_io_num = scl,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .trans_queue_depth = 0,
        .flags = {
            .enable_internal_pullup = true,
        },
    };

    esp_err_t ret = i2c_new_master_bus(&bus_config, &mgr->bus_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create I2C master bus: %s", esp_err_to_name(ret));
        vSemaphoreDelete(mgr->mutex);
        mgr->mutex = NULL;
        return ret;
    }

    // Add TCA9548A device
    // scl_wait_us: hardware SCL-stuck timeout (20ms).  Without this the ESP-IDF
    // I2C peripheral uses its register default (~5 s on ESP32-S3), causing
    // i2c_master_transmit() to block far beyond its xfer_timeout_ms when the
    // bus is glitched by RS485 power-supply transients.
    i2c_device_config_t tca_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = tca_addr,
        .scl_speed_hz = freq_hz,
        .scl_wait_us = 20000,
    };

    ret = i2c_master_bus_add_device(mgr->bus_handle, &tca_config, &mgr->tca_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add TCA9548A device: %s", esp_err_to_name(ret));
        i2c_del_master_bus(mgr->bus_handle);
        vSemaphoreDelete(mgr->mutex);
        mgr->mutex = NULL;
        return ret;
    }

    // Disable all channels initially
    uint8_t disable_cmd = 0x00;
    ret = i2c_master_transmit(mgr->tca_handle, &disable_cmd, 1, pdMS_TO_TICKS(100));
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to disable TCA channels: %s", esp_err_to_name(ret));
        // Non-fatal, continue
    }

    ESP_LOGI(TAG, "I2C mux manager initialized: SDA=%d, SCL=%d, freq=%luHz, TCA@0x%02X",
             sda, scl, (unsigned long)freq_hz, tca_addr);

    return ESP_OK;
}

/**
 * @brief Internal function to select TCA channel (must be called with mutex held)
 */
static esp_err_t select_channel_internal(i2c_mux_mgr_t *mgr, uint8_t channel)
{
    if (channel > 7) {
        return ESP_ERR_INVALID_ARG;
    }

    // Optimization: skip if already on the same channel
    if (mgr->cached_channel == (int8_t)channel) {
        return ESP_OK;
    }

    uint8_t channel_mask = (1 << channel);
    esp_err_t ret = i2c_master_transmit(mgr->tca_handle, &channel_mask, 1, pdMS_TO_TICKS(10));

    if (ret == ESP_OK) {
        mgr->cached_channel = (int8_t)channel;
    } else {
        mgr->cached_channel = -1;  // Invalidate cache on error
        ESP_LOGW(TAG, "TCA channel select failed: ch=%d, err=%s", channel, esp_err_to_name(ret));
    }

    return ret;
}

esp_err_t i2c_mux_mgr_exec(i2c_mux_mgr_t *mgr, uint8_t channel,
                           i2c_mux_exec_fn_t fn, void *ctx)
{
    if (mgr == NULL || fn == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (channel > 7) {
        ESP_LOGE(TAG, "Invalid channel: %d", channel);
        return ESP_ERR_INVALID_ARG;
    }

    // Acquire mutex
    if (xSemaphoreTake(mgr->mutex, pdMS_TO_TICKS(MUX_MUTEX_TIMEOUT_MS)) != pdTRUE) {
        ESP_LOGE(TAG, "Mutex timeout");
        return ESP_ERR_TIMEOUT;
    }

    // Fast-path: if another task is currently running bus_reset outside the mutex,
    // bail immediately instead of proceeding on a bus that is mid-recovery.
    if (mgr->bus_recovering) {
        xSemaphoreGive(mgr->mutex);
        return ESP_ERR_TIMEOUT;
    }

    // Select channel then execute — both failures feed the same health tracker below.
    // IMPORTANT: do NOT early-return on channel-select failure; the recovery logic
    // must see TCA failures too, otherwise a stuck TCA never triggers bus_reset.
    esp_err_t ret = select_channel_internal(mgr, channel);
    if (ret == ESP_OK) {
        ret = fn(ctx);
    }

    // Bus health tracking: success resets the counter; failure may trigger recovery.
    if (ret == ESP_OK) {
        mgr->consec_err_count = 0;
        xSemaphoreGive(mgr->mutex);
    } else {
        mgr->consec_err_count++;
        if (mgr->consec_err_count >= I2C_RECOVERY_THRESHOLD) {
            mgr->total_recovery_count++;
            uint32_t rec_num = mgr->total_recovery_count;
            mgr->consec_err_count = 0;
            mgr->cached_channel = -1;

            // Release the mutex BEFORE calling bus_reset.
            //
            // Previously bus_reset ran while holding the mutex, which blocked
            // the HMI/LCD task (priority 4) for the full reset duration (can be
            // hundreds of ms on a stuck bus), causing the display to freeze.
            //
            // Releasing here lets the HMI task proceed with LCD writes.  Any
            // task that grabs the mutex while the reset is in progress will see
            // bus_recovering == true and return ESP_ERR_TIMEOUT immediately,
            // preventing concurrent i2c_master_transmit during the reset.
            mgr->bus_recovering = true;
            xSemaphoreGive(mgr->mutex);

            ESP_LOGW(TAG, "I2C bus recovery #%lu triggered (consec_err=%lu) — calling bus_reset",
                     (unsigned long)rec_num,
                     (unsigned long)I2C_RECOVERY_THRESHOLD);

            esp_err_t rc = i2c_master_bus_reset(mgr->bus_handle);
            if (rc == ESP_OK) {
                ESP_LOGI(TAG, "Bus reset OK — channel cache invalidated");
            } else {
                ESP_LOGE(TAG, "Bus reset failed: %s — hardware may need power cycle",
                         esp_err_to_name(rc));
            }

            // Clear the recovery flag.  Write is visible to other cores because
            // the flag is volatile; no mutex needed for this single-writer clear.
            mgr->bus_recovering = false;

        } else {
            xSemaphoreGive(mgr->mutex);
        }
    }

    return ret;
}

esp_err_t i2c_mux_mgr_add_device(i2c_mux_mgr_t *mgr, uint8_t dev_addr,
                                  i2c_master_dev_handle_t *dev_handle)
{
    if (mgr == NULL || dev_handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = dev_addr,
        .scl_speed_hz = 100000,  // 100kHz for stability
        .scl_wait_us = 20000,    // 20ms hardware SCL-stuck timeout (see TCA config above)
    };

    esp_err_t ret = i2c_master_bus_add_device(mgr->bus_handle, &dev_config, dev_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add device @0x%02X: %s", dev_addr, esp_err_to_name(ret));
    }

    return ret;
}

i2c_master_bus_handle_t i2c_mux_mgr_get_bus(i2c_mux_mgr_t *mgr)
{
    return (mgr != NULL) ? mgr->bus_handle : NULL;
}

esp_err_t i2c_mux_mgr_deinit(i2c_mux_mgr_t *mgr)
{
    if (mgr == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (mgr->tca_handle != NULL) {
        i2c_master_bus_rm_device(mgr->tca_handle);
        mgr->tca_handle = NULL;
    }

    if (mgr->bus_handle != NULL) {
        i2c_del_master_bus(mgr->bus_handle);
        mgr->bus_handle = NULL;
    }

    if (mgr->mutex != NULL) {
        vSemaphoreDelete(mgr->mutex);
        mgr->mutex = NULL;
    }

    mgr->cached_channel = -1;

    return ESP_OK;
}
