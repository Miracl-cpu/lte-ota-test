/**
 * @file wtvb01_485.h
 * @brief Public API for RS-WZ3 / WTVB01-485 RS485 vibration sensor component
 *
 * Hardware configuration:
 *   - UART1, TX=GPIO17, RX=GPIO18, RTS/DE=GPIO19
 *   - Model-specific baud rate, 8N1, RS485 half-duplex
 *   - Modbus address 0x01
 *
 * Usage:
 * @code
 *   wtvb01_init();
 *
 *   wtvb01_data_t data;
 *   wtvb01_read_all(&data);
 *   printf("Velocity X: %.2f mm/s\n", data.velocity_x);
 * @endcode
 */

#pragma once

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* --------------------------------------------------------------------------
 * Hardware configuration (compile-time)
 * -------------------------------------------------------------------------- */

/** Supported vibration sensor models. RS-WZ3 is the default. */
typedef enum {
  WTVB01_SENSOR_MODEL_RS_WZ3 = 0,
  WTVB01_SENSOR_MODEL_WTVB01_485 = 1,
} wtvb01_sensor_model_t;

#define WTVB01_UART_NUM 1          /**< UART port used */
#define WTVB01_TX_PIN 17           /**< GPIO for UART TX (to MAX485 DI) */
#define WTVB01_RX_PIN 18           /**< GPIO for UART RX (from MAX485 RO) */
#define WTVB01_RTS_PIN 19          /**< GPIO for MAX485 DE/RE direction control */
#define RS_WZ3_BAUD_RATE 4800      /**< RS-WZ3 factory default baud rate */
#define WTVB01_485_BAUD_RATE 9600  /**< WTVB01-485 normal-mode factory default baud rate */

/** Response timeout in milliseconds
 *  At 4800 baud: 31-byte response takes ~65ms + sensor processing.
 *  200ms gives comfortable margin at any supported baud rate. */
#define WTVB01_RESPONSE_TIMEOUT_MS 1000

/** Background task stack size in bytes */
#define WTVB01_TASK_STACK_SIZE 8192

/** Background task FreeRTOS priority */
#define WTVB01_TASK_PRIORITY (tskIDLE_PRIORITY + 2)

/** Console log interval: print data every N milliseconds */
#define WTVB01_LOG_INTERVAL_MS 1000

/**
 * Log mode selection (change this macro to switch console output)
 *   0 = RAW     – prints all 19 raw 16-bit register values (integer, 1 Hz)
 *   1 = PARSED  – prints named parameters with units (multi-line, 1 Hz)
 *   2 = PLOTTER – single CSV line per read for serial plotter (1 Hz)
 *   3 = STATS   – prints 1-min stats summary (livedata fields, every 60 s)
 *   4 = SILENT  – no background logging, main app handles it
 */
#define WTVB01_LOG_MODE 1

/** RS-WZ3 sensor polling interval in ms */
#define WTVB01_POLL_INTERVAL_MS 1000

/** WTVB01-485 field profile from the WitMotion PC tool. These are sensor-side
 * settings, not RS-WZ3 settings. Configure them in the PC tool before use. */
#define WTVB01_485_PROFILE_SAMPLING_HZ       256
#define WTVB01_485_PROFILE_FUNDAMENTAL_HZ    50
#define WTVB01_485_PROFILE_ALGORITHM_NAME    "frequency-domain algorithm"
#define WTVB01_485_PROFILE_DISP_RANGE_UM     60000
#define WTVB01_485_PROFILE_DISP_RES_UM       1
#define WTVB01_485_POLL_INTERVAL_MS          100

/* --------------------------------------------------------------------------
 * Sensor data structure
 * -------------------------------------------------------------------------- */

/**
 * @brief All sensor readings and derived values
 *
 * Updated continuously by the background task.
 * Access via wtvb01_read_all() for a thread-safe copy.
 */
typedef struct {
  /* Vibration Velocity (mm/s) - PRIORITY 1 */
  float velocity_x;   /**< X-axis velocity [mm/s] */
  float velocity_y;   /**< Y-axis velocity [mm/s] */
  float velocity_z;   /**< Z-axis velocity [mm/s] */
  float velocity_rms; /**< RMS velocity sqrt(vx²+vy²+vz²) [mm/s] */

  /* Vibration Displacement (μm) */
  float displacement_x; /**< X-axis displacement [μm] */
  float displacement_y; /**< Y-axis displacement [μm] */
  float displacement_z; /**< Z-axis displacement [μm] */

  /* Acceleration (g) */
  float accel_x;   /**< X-axis acceleration [g] */
  float accel_y;   /**< Y-axis acceleration [g] */
  float accel_z;   /**< Z-axis acceleration [g] */
  float accel_rms; /**< RMS acceleration sqrt(ax²+ay²+az²) [g] */

  /* Vibration Frequency (Hz) */
  float frequency_x;        /**< X-axis dominant frequency [Hz] */
  float frequency_y;        /**< Y-axis dominant frequency [Hz] */
  float frequency_z;        /**< Z-axis dominant frequency [Hz] */
  float dominant_frequency; /**< max(fx, fy, fz) [Hz] */

  /* Angular Velocity (°/s) */
  float angular_vel_x; /**< X-axis angular velocity [°/s] */
  float angular_vel_y; /**< Y-axis angular velocity [°/s] */
  float angular_vel_z; /**< Z-axis angular velocity [°/s] */

  /* Inclination Angle (degrees) */
  float angle_x; /**< X tilt angle [°] range ±180 */
  float angle_y; /**< Y tilt angle [°] range ±90 */
  float angle_z; /**< Z tilt angle [°] range ±180 */

  /* Temperature */
  float temperature; /**< Sensor temperature [°C] */

  /* Status */
  bool data_valid;      /**< true once at least one read succeeded */
  uint32_t read_count;  /**< Total successful full reads */
  uint32_t error_count; /**< Total communication errors */

} wtvb01_data_t;

/* --------------------------------------------------------------------------
 * Statistics structure (for 1-minute livedata reporting)
 * -------------------------------------------------------------------------- */

/**
 * @brief Accumulated statistics over a reporting window.
 *
 * Call wtvb01_get_stats() to retrieve and optionally reset.
 * Designed for IoT gateways that send livedata every 1 minute.
 */
typedef struct {
  /* Velocity RMS (mm/s) — primary health indicator */
  float velocity_rms_avg; /**< Average velocity RMS over window */
  float velocity_rms_max; /**< Peak velocity RMS (catches transients) */

  /* Acceleration RMS (g) — impact/shock detection */
  float accel_rms_avg; /**< Average acceleration RMS */
  float accel_rms_max; /**< Peak acceleration RMS */

  /* Displacement average (μm) — alignment/balance */
  float displacement_x_avg;
  float displacement_y_avg;
  float displacement_z_avg;

  /* Frequency (Hz) — fault classification */
  float frequency_x_avg;
  float frequency_y_avg;
  float frequency_z_avg;
  float dominant_freq_avg; /**< Average dominant frequency */

  /* Temperature — latest value (slow-changing) */
  float temperature; /**< Latest temperature reading [°C] */

  /* Window info */
  uint32_t sample_count; /**< Number of successful reads in window */
  uint32_t error_count;  /**< Number of failed reads in window */
} wtvb01_stats_t;

/* --------------------------------------------------------------------------
 * Public API
 * -------------------------------------------------------------------------- */

/**
 * @brief Initialize the RS-WZ3 / WTVB01-485 component
 *
 * Configures UART1, creates the mutex and starts the background reading task.
 * Safe to call once at startup.
 *
 * @return ESP_OK on success, ESP_FAIL or ESP_ERR_* on failure
 */
esp_err_t wtvb01_init(void);

/**
 * @brief Select the vibration sensor register map/parser.
 *
 * Safe to call before or after init. If called while running, latest data and
 * statistics are cleared so old-model values are not mixed with new-model data.
 */
esp_err_t wtvb01_set_sensor_model(wtvb01_sensor_model_t model);

/**
 * @brief Get the currently selected vibration sensor model.
 */
wtvb01_sensor_model_t wtvb01_get_sensor_model(void);

/**
 * @brief Human-readable model name for logs/UI.
 */
const char *wtvb01_sensor_model_name(wtvb01_sensor_model_t model);

/**
 * @brief De-initialize the component and free all resources
 *
 * Stops the background task, deletes the mutex, and uninstalls the UART driver.
 *
 * @return ESP_OK on success
 */
esp_err_t wtvb01_deinit(void);

/**
 * @brief Get a thread-safe copy of the latest sensor data
 *
 * Blocks only for the brief mutex acquisition; does NOT trigger a new read.
 *
 * @param[out] data Pointer to caller-allocated structure to fill
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG if data is NULL,
 *         ESP_ERR_INVALID_STATE if not initialised
 */
esp_err_t wtvb01_read_all(wtvb01_data_t *data);

/**
 * @brief Convenience function: get latest vibration velocity values
 *
 * @param[out] vx  X-axis velocity [mm/s] (may be NULL to skip)
 * @param[out] vy  Y-axis velocity [mm/s] (may be NULL to skip)
 * @param[out] vz  Z-axis velocity [mm/s] (may be NULL to skip)
 * @return ESP_OK, or ESP_ERR_INVALID_STATE if not initialised / no valid data
 */
esp_err_t wtvb01_read_velocity(float *vx, float *vy, float *vz);

/**
 * @brief Convenience function: get latest temperature reading
 *
 * @param[out] temp_c Temperature [°C] (must not be NULL)
 * @return ESP_OK, or ESP_ERR_INVALID_STATE if not initialised / no valid data
 */
esp_err_t wtvb01_read_temperature(float *temp_c);

/**
 * @brief Get accumulated statistics and optionally reset the window.
 *
 * Returns avg/peak values computed over all reads since last reset.
 * Designed to be called once per livedata cycle (e.g. every 1 minute).
 *
 * @param[out] stats  Pointer to caller-allocated stats structure
 * @param      reset  If true, reset accumulators after copying
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG if stats is NULL,
 *         ESP_ERR_INVALID_STATE if not initialised or no samples yet
 */
esp_err_t wtvb01_get_stats(wtvb01_stats_t *stats, bool reset);

/**
 * @brief Write calibration factor (Speed Coefficient A) to sensor register
 *
 * Speed Coefficient A affects vibration velocity measurement.
 * Requires password authentication (7552).
 *
 * @param axis 'X', 'Y', or 'Z' axis selector
 * @param factor Calibration factor value (float, stored as uint32)
 * @return ESP_OK on success, ESP_ERR_* on failure
 */
esp_err_t wtvb01_write_calibration_factor(char axis, float factor);

/**
 * @brief Write calibration offset (Speed Coefficient B) to sensor register
 *
 * Speed Coefficient B (offset) affects vibration velocity measurement.
 * Does NOT require password authentication.
 *
 * @param axis 'X', 'Y', or 'Z' axis selector
 * @param offset Calibration offset value (float, stored as uint32)
 * @return ESP_OK on success, ESP_ERR_* on failure
 */
esp_err_t wtvb01_write_calibration_offset(char axis, float offset);

/**
 * @brief Read current calibration values for all axes
 *
 * @param[out] x_factor X-axis factor (may be NULL)
 * @param[out] x_offset X-axis offset (may be NULL)
 * @param[out] y_factor Y-axis factor (may be NULL)
 * @param[out] y_offset Y-axis offset (may be NULL)
 * @param[out] z_factor Z-axis factor (may be NULL)
 * @param[out] z_offset Z-axis offset (may be NULL)
 * @return ESP_OK on success, ESP_ERR_* on failure
 */
esp_err_t wtvb01_read_calibration_all(float *x_factor, float *x_offset,
                                       float *y_factor, float *y_offset,
                                       float *z_factor, float *z_offset);

#ifdef __cplusplus
}
#endif
