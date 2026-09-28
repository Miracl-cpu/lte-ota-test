/**
 * @file daq_ct_ads1115.h
 * @brief CT Current Data Acquisition using ADS1115 ADC
 *
 * Features:
 * - 3-phase CT current measurement (AIN0, AIN1, AIN2)
 * - Unipolar RMS calculation (sqrt(E[v^2]))
 * - CT open detection via high-impedance voltage detection
 * - Dummy conversion for MUX ghosting avoidance
 * - Thread-safe snapshot access
 */

#ifndef DAQ_CT_ADS1115_H
#define DAQ_CT_ADS1115_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "i2c_mux_mgr.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief DAQ snapshot structure containing latest measurements
 */
typedef struct {
    uint32_t seq;           ///< Sequence number
    int64_t  ts_ms;         ///< Timestamp in milliseconds
    float ia_rms;           ///< Phase A RMS current (Amps)
    float ib_rms;           ///< Phase B RMS current (Amps)
    float ic_rms;           ///< Phase C RMS current (Amps)
    uint8_t ct_open_mask;   ///< CT open flags: bit0=A, bit1=B, bit2=C
    float va_rms;           ///< Phase A voltage RMS (debug)
    float vb_rms;           ///< Phase B voltage RMS (debug)
    float vc_rms;           ///< Phase C voltage RMS (debug)
    float va_mean;          ///< Phase A mean voltage (debug)
    float vb_mean;          ///< Phase B mean voltage (debug)
    float vc_mean;          ///< Phase C mean voltage (debug)
    uint32_t i2c_err_count; ///< Cumulative I2C error count
    uint32_t ads_err_count; ///< Cumulative ADS1115 error count
} daq_snapshot_t;

/**
 * @brief DAQ configuration structure
 */
typedef struct {
    i2c_mux_mgr_t *mux_mgr;     ///< Pointer to mux manager
    uint8_t ads_addr;           ///< ADS1115 I2C address (0x48)
    uint8_t tca_channel;        ///< TCA9548A channel (7)
    float cal_factor[3];        ///< Per-phase calibration factors
    int alert_gpio;             ///< GPIO pin for ADS1115 ALERT/RDY (-1 = polling fallback)
    float ct_amps_per_vrms;     ///< CT ratio: Amps per Vrms (e.g. 100.0 for 100A/1V CT; 0 = use default 100.0)
} daq_config_t;

/**
 * @brief DAQ handle structure (opaque)
 */
typedef struct daq_ct_handle_s daq_ct_handle_t;

/**
 * @brief Initialize the DAQ CT system
 *
 * @param config Pointer to configuration
 * @param handle Output: DAQ handle
 * @return ESP_OK on success
 */
esp_err_t daq_ct_init(const daq_config_t *config, daq_ct_handle_t **handle);

/**
 * @brief DAQ task function (for FreeRTOS)
 *
 * This task runs the continuous acquisition loop.
 * Pass the handle as task argument.
 *
 * @param arg DAQ handle pointer
 */
void daq_ct_task(void *arg);

/**
 * @brief Get the latest DAQ snapshot (thread-safe)
 *
 * @param handle DAQ handle
 * @param out Output: snapshot copy
 */
void daq_ct_get_latest(daq_ct_handle_t *handle, daq_snapshot_t *out);

/**
 * @brief Get DAQ handle for global access (set during init)
 *
 * @return DAQ handle or NULL if not initialized
 */
daq_ct_handle_t *daq_ct_get_handle(void);

/**
 * @brief Update calibration factors on a running DAQ handle (thread-safe, takes effect immediately)
 *
 * Replaces each cal_factor[i] where cal[i] > 0. Pass 0 for any phase to leave it unchanged.
 *
 * @param handle DAQ handle
 * @param cal    Array of 3 calibration factors (per-phase)
 * @return ESP_OK on success
 */
esp_err_t daq_ct_update_cal_factors(daq_ct_handle_t *handle, const float cal[3]);

/**
 * @brief Deinitialize DAQ
 *
 * @param handle DAQ handle
 * @return ESP_OK on success
 */
esp_err_t daq_ct_deinit(daq_ct_handle_t *handle);

#ifdef __cplusplus
}
#endif

#endif // DAQ_CT_ADS1115_H
