/**
 * @file i2c_mux_mgr.h
 * @brief I2C Multiplexer Manager for TCA9548A with mutex protection
 *
 * This module owns the I2C bus handle and TCA9548A device, providing
 * thread-safe channel selection and transaction execution.
 */

#ifndef I2C_MUX_MGR_H
#define I2C_MUX_MGR_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief I2C Mux Manager handle structure
 */
typedef struct {
    i2c_master_bus_handle_t bus_handle;     ///< I2C master bus handle
    i2c_master_dev_handle_t tca_handle;     ///< TCA9548A device handle
    SemaphoreHandle_t mutex;                 ///< Mutex for thread safety
    uint8_t tca_addr;                        ///< TCA9548A I2C address
    int8_t cached_channel;                   ///< Currently selected channel (-1 = unknown)
    uint32_t consec_err_count;              ///< Consecutive exec errors — triggers bus recovery
    uint32_t total_recovery_count;          ///< Total i2c_master_bus_reset() calls made
    volatile bool bus_recovering;           ///< True while bus_reset is running outside the mutex
} i2c_mux_mgr_t;

/**
 * @brief Callback function type for mux_exec operations
 * @param ctx User context pointer
 * @return ESP_OK on success, error code otherwise
 */
typedef esp_err_t (*i2c_mux_exec_fn_t)(void *ctx);

/**
 * @brief Initialize the I2C mux manager
 *
 * Creates I2C master bus, adds TCA9548A device, and initializes mutex.
 *
 * @param mgr Pointer to mux manager structure (caller allocated)
 * @param sda SDA GPIO number
 * @param scl SCL GPIO number
 * @param freq_hz I2C frequency in Hz (recommend 100000)
 * @param tca_addr TCA9548A I2C address (typically 0x70)
 * @return ESP_OK on success
 */
esp_err_t i2c_mux_mgr_init(i2c_mux_mgr_t *mgr, gpio_num_t sda, gpio_num_t scl,
                           uint32_t freq_hz, uint8_t tca_addr);

/**
 * @brief Execute a function with exclusive I2C bus access on specified TCA channel
 *
 * This function:
 * 1. Acquires the mutex
 * 2. Selects the specified TCA channel
 * 3. Calls the user function
 * 4. Releases the mutex
 *
 * @param mgr Pointer to initialized mux manager
 * @param channel TCA9548A channel (0-7)
 * @param fn User function to execute
 * @param ctx User context passed to fn
 * @return ESP_OK if channel selection and fn succeeded, otherwise first error
 */
esp_err_t i2c_mux_mgr_exec(i2c_mux_mgr_t *mgr, uint8_t channel,
                           i2c_mux_exec_fn_t fn, void *ctx);

/**
 * @brief Add a device to the I2C bus (for use by downstream components)
 *
 * @param mgr Pointer to initialized mux manager
 * @param dev_addr 7-bit I2C device address
 * @param dev_handle Output: device handle
 * @return ESP_OK on success
 */
esp_err_t i2c_mux_mgr_add_device(i2c_mux_mgr_t *mgr, uint8_t dev_addr,
                                  i2c_master_dev_handle_t *dev_handle);

/**
 * @brief Get the I2C bus handle (for advanced use)
 *
 * @param mgr Pointer to initialized mux manager
 * @return Bus handle
 */
i2c_master_bus_handle_t i2c_mux_mgr_get_bus(i2c_mux_mgr_t *mgr);

/**
 * @brief Deinitialize the mux manager
 *
 * @param mgr Pointer to mux manager
 * @return ESP_OK on success
 */
esp_err_t i2c_mux_mgr_deinit(i2c_mux_mgr_t *mgr);

#ifdef __cplusplus
}
#endif

#endif // I2C_MUX_MGR_H
