/**
 * @file wtvb01_485.c
 * @brief RS-WZ3 / WTVB01-485 vibration sensor driver implementation
 *
 * Implements:
 *   - UART1 initialisation (4800 baud, 8N1, RS485 RTS/DE control)
 *   - Modbus RTU communication with timing delays
 *   - Batch register reads for all sensor parameters
 *   - FreeRTOS background task with mutex-protected shared data
 *   - Public API (init, deinit, read_all, read_velocity, read_temperature)
 */

#include "wtvb01_485.h"
#include "wtvb01_modbus.h"

#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "log_config.h"
#include "rs485_bus.h"
#undef  LOG_LOCAL_LEVEL
#define LOG_LOCAL_LEVEL LOG_LEVEL_VIB
#include "esp_log.h"
#include "esp_task_wdt.h"

static const char *TAG = "VIB_RS485";

/* --------------------------------------------------------------------------
 * Timing constants (baud-rate aware)
 * -------------------------------------------------------------------------- */

#define PRE_TX_DELAY_US         100  /**< Safety margin before transmitting */
#define POST_TX_DELAY_US        200  /**< Wait after last byte leaves UART FIFO */

/* UART receive buffer size */
#define UART_BUF_SIZE           512

/* --------------------------------------------------------------------------
 * Module-level state
 * -------------------------------------------------------------------------- */

static SemaphoreHandle_t  s_mutex      = NULL;
static TaskHandle_t       s_task       = NULL;
static wtvb01_data_t      s_data       = {0};
static volatile bool      s_running    = false;
static volatile wtvb01_sensor_model_t s_model = WTVB01_SENSOR_MODEL_RS_WZ3;

static int model_baud_rate(wtvb01_sensor_model_t model)
{
    return (model == WTVB01_SENSOR_MODEL_WTVB01_485)
               ? WTVB01_485_BAUD_RATE
               : RS_WZ3_BAUD_RATE;
}

static uint8_t model_modbus_addr(wtvb01_sensor_model_t model)
{
    return (model == WTVB01_SENSOR_MODEL_WTVB01_485)
               ? WTVB01_485_MODBUS_ADDR
               : RS_WZ3_MODBUS_ADDR;
}

static uint32_t model_inter_frame_delay_us(wtvb01_sensor_model_t model)
{
    int baud = model_baud_rate(model);
    uint32_t char_time_us = (uint32_t)((10000000UL + (uint32_t)baud - 1) /
                                       (uint32_t)baud);
    return char_time_us * 4U;
}

static uint32_t model_poll_interval_ms(wtvb01_sensor_model_t model)
{
    return (model == WTVB01_SENSOR_MODEL_WTVB01_485)
               ? WTVB01_485_POLL_INTERVAL_MS
               : WTVB01_POLL_INTERVAL_MS;
}

static void log_wtvb01_profile_if_needed(wtvb01_sensor_model_t model)
{
    if (model != WTVB01_SENSOR_MODEL_WTVB01_485) {
        return;
    }

    ESP_LOGI(TAG,
             "WTVB01-485 PC profile: sampling=%uHz, "
             "spectrum_fundamental=%uHz, algorithm=%s, "
             "displacement=%uum/%uum",
             (unsigned)WTVB01_485_PROFILE_SAMPLING_HZ,
             (unsigned)WTVB01_485_PROFILE_FUNDAMENTAL_HZ,
             WTVB01_485_PROFILE_ALGORITHM_NAME,
             (unsigned)WTVB01_485_PROFILE_DISP_RANGE_UM,
             (unsigned)WTVB01_485_PROFILE_DISP_RES_UM);
}

static esp_err_t apply_model_uart_settings(wtvb01_sensor_model_t model)
{
    esp_err_t ret = rs485_bus_set_baudrate(model_baud_rate(model));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set %s baud %d: %s",
                 wtvb01_sensor_model_name(model), model_baud_rate(model),
                 esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "%s stream selected: UART%d %d baud addr 0x%02X poll %lums",
             wtvb01_sensor_model_name(model), WTVB01_UART_NUM,
             model_baud_rate(model), model_modbus_addr(model),
             (unsigned long)model_poll_interval_ms(model));
    log_wtvb01_profile_if_needed(model);
    return ESP_OK;
}


/* --------------------------------------------------------------------------
 * Statistics accumulator (internal)
 * -------------------------------------------------------------------------- */

typedef struct {
    /* Running sums for average computation */
    double sum_vel_rms;
    double sum_accel_rms;
    double sum_disp_x, sum_disp_y, sum_disp_z;
    double sum_freq_x, sum_freq_y, sum_freq_z;
    double sum_dom_freq;

    /* Peak values */
    float  max_vel_rms;
    float  max_accel_rms;

    /* Latest temperature (slow-changing, no need to average) */
    float  last_temp;

    /* Counters */
    uint32_t sample_count;
    uint32_t error_count;
} stats_accum_t;

static stats_accum_t s_accum = {0};

/** Reset accumulator to zero */
static void stats_reset(void)
{
    memset(&s_accum, 0, sizeof(s_accum));
}

/** Accumulate one data sample into the running statistics */
static void stats_accumulate(const wtvb01_data_t *d)
{
    s_accum.sum_vel_rms   += d->velocity_rms;
    s_accum.sum_accel_rms += d->accel_rms;
    s_accum.sum_disp_x    += d->displacement_x;
    s_accum.sum_disp_y    += d->displacement_y;
    s_accum.sum_disp_z    += d->displacement_z;
    s_accum.sum_freq_x    += d->frequency_x;
    s_accum.sum_freq_y    += d->frequency_y;
    s_accum.sum_freq_z    += d->frequency_z;
    s_accum.sum_dom_freq  += d->dominant_frequency;

    if (d->velocity_rms > s_accum.max_vel_rms) {
        s_accum.max_vel_rms = d->velocity_rms;
        /* NOTE: do NOT log here — this function is called while holding s_mutex,
         * and ESP_LOGI acquires the log lock. Log the peak from outside the mutex. */
    }
    if (d->accel_rms > s_accum.max_accel_rms)
        s_accum.max_accel_rms = d->accel_rms;

    s_accum.last_temp = d->temperature;
    s_accum.sample_count++;
}

/* --------------------------------------------------------------------------
 * Low-level UART helpers
 * -------------------------------------------------------------------------- */

/**
 * @brief Send a Modbus request and receive the response.
 *
 * Sequence:
 *   1. Flush RX buffer (discard stale bytes)
 *   2. PRE_TX_DELAY_US guard
 *   3. Write request bytes
 *   4. Wait for hardware FIFO drain + POST_TX_DELAY_US
 *   5. Read expected response bytes with WTVB01_RESPONSE_TIMEOUT_MS
 *
 * @param cmd          Request buffer (8 bytes)
 * @param cmd_len      Length of request (should be MODBUS_REQUEST_LEN)
 * @param resp         Output buffer for response
 * @param expected_len Exact number of response bytes expected
 * @return Number of bytes actually received
 */
static int uart_transact(const uint8_t *cmd, int cmd_len,
                         uint8_t *resp, int expected_len)
{
    wtvb01_sensor_model_t model = s_model;
    int received = 0;
    esp_err_t ret = rs485_bus_transact(
        model_baud_rate(model), cmd, cmd_len, resp, expected_len,
        WTVB01_RESPONSE_TIMEOUT_MS, PRE_TX_DELAY_US, POST_TX_DELAY_US,
        model_inter_frame_delay_us(model), &received);
    if (ret != ESP_OK) {
        return 0;
    }
    return received;

#if 0
    /* Flush any stale bytes in the RX FIFO */
    uart_flush_input(WTVB01_UART_NUM);

    /* Guard time: ensure bus is idle before transmitting */
    esp_rom_delay_us(PRE_TX_DELAY_US);

    /* Transmit request */
    uart_write_bytes(WTVB01_UART_NUM, (const char *)cmd, cmd_len);

    /* Wait for all bytes to leave the hardware FIFO.
     * At 4800 baud (RS_WZ3), 8 bytes × 10 bits = 80 bits → ~16.7 ms.
     * 25 ms gives comfortable margin at the lowest supported baud rate. */
    uart_wait_tx_done(WTVB01_UART_NUM, pdMS_TO_TICKS(25));
    esp_rom_delay_us(POST_TX_DELAY_US);

    /* Read response */
    int received = uart_read_bytes(WTVB01_UART_NUM, resp, expected_len,
                                   pdMS_TO_TICKS(WTVB01_RESPONSE_TIMEOUT_MS));

    /* Enforce inter-frame gap before next transaction.
     * Yield CPU via vTaskDelay instead of busy-waiting (RS_WZ3: ~7.3 ms). */
    vTaskDelay(pdMS_TO_TICKS((model_inter_frame_delay_us(model) + 999) / 1000));

    return received;
#endif
}

/**
 * @brief Read Modbus holding registers from the sensor.
 *
 * Builds the request, sends it, validates the response, and fills @p regs.
 *
 * @param start_reg Starting register address
 * @param num_regs  Number of registers to read
 * @param regs      Output array (must hold at least num_regs elements)
 * @return ESP_OK on success, ESP_FAIL on any communication error
 */
static esp_err_t read_registers_for_model(wtvb01_sensor_model_t model,
                                          uint16_t start_reg,
                                          uint16_t num_regs,
                                          uint16_t *regs)
{
    uint8_t cmd[MODBUS_REQUEST_LEN];
    uint8_t resp[MODBUS_MAX_RESPONSE_LEN];
    uint8_t addr = model_modbus_addr(model);

    modbus_build_read_cmd_addr(addr, cmd, start_reg, num_regs);

    int expected = modbus_response_len(num_regs);
    int received = uart_transact(cmd, MODBUS_REQUEST_LEN, resp, expected);

    if (received != expected) {
        ESP_LOGW(TAG, "%s timeout addr 0x%02X reg 0x%04X: expected %d bytes, got %d",
                 wtvb01_sensor_model_name(model), addr, start_reg, expected,
                 received);
        if (received > 0) {
            ESP_LOG_BUFFER_HEX(TAG, resp, received);
        }
        return ESP_FAIL;
    }

    int parsed = modbus_parse_response_addr(addr, resp, (size_t)received, regs,
                                            num_regs);
    if (parsed < 0) {
        ESP_LOGW(TAG, "%s parse error %d for addr 0x%02X reg 0x%04X",
                 wtvb01_sensor_model_name(model), parsed, addr, start_reg);
        ESP_LOG_BUFFER_HEX(TAG, resp, received);
        return ESP_FAIL;
    }

    return ESP_OK;
}

const char *wtvb01_sensor_model_name(wtvb01_sensor_model_t model)
{
    switch (model) {
        case WTVB01_SENSOR_MODEL_WTVB01_485:
            return "WTVB01-485";
        case WTVB01_SENSOR_MODEL_RS_WZ3:
        default:
            return "RS-WZ3";
    }
}

wtvb01_sensor_model_t wtvb01_get_sensor_model(void)
{
    return s_model;
}

esp_err_t wtvb01_set_sensor_model(wtvb01_sensor_model_t model)
{
    if (model != WTVB01_SENSOR_MODEL_RS_WZ3 &&
        model != WTVB01_SENSOR_MODEL_WTVB01_485) {
        return ESP_ERR_INVALID_ARG;
    }

    if (s_model == model) {
        return ESP_OK;
    }

    if (s_running) {
        esp_err_t uart_ret = apply_model_uart_settings(model);
        if (uart_ret != ESP_OK) {
            return uart_ret;
        }
    }

    if (s_mutex != NULL && xSemaphoreTake(s_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        s_model = model;
        memset(&s_data, 0, sizeof(s_data));
        stats_reset();
        xSemaphoreGive(s_mutex);
    } else if (s_running) {
        return ESP_ERR_TIMEOUT;
    } else {
        s_model = model;
        memset(&s_data, 0, sizeof(s_data));
        stats_reset();
    }

    ESP_LOGI(TAG, "Sensor model set to %s (%d baud, addr 0x%02X, poll %lums)",
             wtvb01_sensor_model_name(model), model_baud_rate(model),
             model_modbus_addr(model),
             (unsigned long)model_poll_interval_ms(model));
    log_wtvb01_profile_if_needed(model);
    return ESP_OK;
}

/* --------------------------------------------------------------------------
 * Sensor data acquisition
 * -------------------------------------------------------------------------- */

/**
 * @brief Read all sensor parameters in a single Modbus transaction.
 *
 * @param d        Output data structure to fill
 * @param raw_out  Optional raw register snapshot (min VIB_MAX_REG_DATA_COUNT elements)
 * @return ESP_OK on success, ESP_FAIL on communication error
 */
static esp_err_t sensor_read_all(wtvb01_data_t *d, uint16_t *raw_out)
{
    wtvb01_sensor_model_t model = s_model;
    uint16_t start_reg = (model == WTVB01_SENSOR_MODEL_WTVB01_485)
                             ? WTVB01_485_REG_DATA_START
                             : RS_WZ3_REG_DATA_START;
    uint16_t reg_count = (model == WTVB01_SENSOR_MODEL_WTVB01_485)
                             ? WTVB01_485_REG_DATA_COUNT
                             : RS_WZ3_REG_DATA_COUNT;
    uint16_t regs[VIB_MAX_REG_DATA_COUNT];
    memset(regs, 0, sizeof(regs));

    esp_err_t ret = read_registers_for_model(model, start_reg, reg_count, regs);
    if (ret != ESP_OK) {
        return ESP_FAIL;
    }

    /* Copy raw snapshot for log mode 0 */
    if (raw_out != NULL) {
        memset(raw_out, 0, VIB_MAX_REG_DATA_COUNT * sizeof(uint16_t));
        memcpy(raw_out, regs, reg_count * sizeof(uint16_t));
    }

    if (model == WTVB01_SENSOR_MODEL_WTVB01_485) {
    /* WTVB01-485 normal-mode map from the WitMotion manual:
     * 0x34..0x36 accel, 0x37..0x39 angular velocity, 0x3A..0x3C
     * vibration speed, 0x3D..0x3F vibration angle, 0x40 temperature,
     * 0x41..0x43 displacement, 0x44..0x46 frequency. */
    /* --- Acceleration (int16, raw / 32768 * 16 → g) --- */
    d->accel_x = (int16_t)regs[0] / 32768.0f * 16.0f;
    d->accel_y = (int16_t)regs[1] / 32768.0f * 16.0f;
    d->accel_z = (int16_t)regs[2] / 32768.0f * 16.0f;
    d->accel_rms = sqrtf(d->accel_x * d->accel_x +
                         d->accel_y * d->accel_y +
                         d->accel_z * d->accel_z);

    /* --- Angular velocity (0x37~0x39 undocumented in datasheet — zero-fill) --- */
    d->angular_vel_x = (int16_t)regs[3] / 32768.0f * 2000.0f;
    d->angular_vel_y = (int16_t)regs[4] / 32768.0f * 2000.0f;
    d->angular_vel_z = (int16_t)regs[5] / 32768.0f * 2000.0f;

    /* --- Vibration velocity (int16, raw / 100 → mm/s) --- */
    d->velocity_x = (float)(int16_t)regs[6] / 100.0f;
    d->velocity_y = (float)(int16_t)regs[7] / 100.0f;
    d->velocity_z = (float)(int16_t)regs[8] / 100.0f;
    d->velocity_rms = sqrtf(d->velocity_x * d->velocity_x +
                            d->velocity_y * d->velocity_y +
                            d->velocity_z * d->velocity_z);

    /* --- Angle registers (0x3D~0x3F are RESERVED in datasheet — zero-fill) --- */
    d->angle_x = (int16_t)regs[9] / 32768.0f * 180.0f;
    d->angle_y = (int16_t)regs[10] / 32768.0f * 180.0f;
    d->angle_z = (int16_t)regs[11] / 32768.0f * 180.0f;

    /* --- Temperature (int16, raw / 100 → °C) --- */
    d->temperature = (int16_t)regs[12] / 100.0f;

    /* --- Displacement (uint16, raw → μm, no scaling) --- */
    d->displacement_x = (float)(int16_t)regs[13];
    d->displacement_y = (float)(int16_t)regs[14];
    d->displacement_z = (float)(int16_t)regs[15];

    /* --- Frequency (uint16, raw / 10 → Hz per datasheet) --- */
    d->frequency_x = (float)regs[16] / 10.0f;
    d->frequency_y = (float)regs[17] / 10.0f;
    d->frequency_z = (float)regs[18] / 10.0f;
    d->dominant_frequency = fmaxf(d->frequency_x,
                                  fmaxf(d->frequency_y, d->frequency_z));

    } else {
    /* RS-WZ3WZ1-N01-1 Parsing */
    /* Regs: [0] TEMP, [1] VX, [2] VY, [3] VZ, [4] DX, [5] DY, [6] DZ, [7]-[9] skipped, [10] AX, [11] AY, [12] AZ */
    d->temperature = (int16_t)regs[0] / 10.0f;

    d->velocity_x = (float)regs[1] / 10.0f;
    d->velocity_y = (float)regs[2] / 10.0f;
    d->velocity_z = (float)regs[3] / 10.0f;
    d->velocity_rms = sqrtf(d->velocity_x * d->velocity_x + d->velocity_y * d->velocity_y + d->velocity_z * d->velocity_z);

    d->displacement_x = (float)regs[4] / 10.0f;
    d->displacement_y = (float)regs[5] / 10.0f;
    d->displacement_z = (float)regs[6] / 10.0f;

    /* Acceleration: raw is m/s2 * 10. To get g, divide by 9.8. So: (raw / 10.0) / 9.8 */
    d->accel_x = ((int16_t)regs[10] / 10.0f) / 9.8f;
    d->accel_y = ((int16_t)regs[11] / 10.0f) / 9.8f;
    d->accel_z = ((int16_t)regs[12] / 10.0f) / 9.8f;
    d->accel_rms = sqrtf(d->accel_x * d->accel_x + d->accel_y * d->accel_y + d->accel_z * d->accel_z);

    /* Unsupported parameters set to 0 */
    d->angular_vel_x = 0; d->angular_vel_y = 0; d->angular_vel_z = 0;
    d->angle_x = 0; d->angle_y = 0; d->angle_z = 0;
    d->frequency_x = 0; d->frequency_y = 0; d->frequency_z = 0;
    d->dominant_frequency = 0;
    }

    return ESP_OK;
}

/* --------------------------------------------------------------------------
 * Logging helper
 * -------------------------------------------------------------------------- */

/**
 * @brief LOG MODE 0 — Print raw 16-bit register values (integer)
 *
 * WTVB01: prints 19 registers. RS-WZ3: prints 13 registers (8 used, 3 skipped).
 * Signed format used to make negative values (accel, temp) visible.
 */
static void log_raw_registers(const uint16_t *raw, uint32_t read_count)
{
    ESP_LOGI(TAG, "=== RAW 16-bit Register Values (read #%lu) ===", (unsigned long)read_count);

    if (s_model == WTVB01_SENSOR_MODEL_WTVB01_485) {
    ESP_LOGI(TAG, "AX  [0x34] : %d",  (int)(int16_t)raw[0]);
    ESP_LOGI(TAG, "AY  [0x35] : %d",  (int)(int16_t)raw[1]);
    ESP_LOGI(TAG, "AZ  [0x36] : %d",  (int)(int16_t)raw[2]);
    ESP_LOGI(TAG, "GX  [0x37] : %d",  (int)(int16_t)raw[3]);
    ESP_LOGI(TAG, "GY  [0x38] : %d",  (int)(int16_t)raw[4]);
    ESP_LOGI(TAG, "GZ  [0x39] : %d",  (int)(int16_t)raw[5]);
    ESP_LOGI(TAG, "VX  [0x3A] : %d",  (int)(int16_t)raw[6]);
    ESP_LOGI(TAG, "VY  [0x3B] : %d",  (int)(int16_t)raw[7]);
    ESP_LOGI(TAG, "VZ  [0x3C] : %d",  (int)(int16_t)raw[8]);
    ESP_LOGI(TAG, "ADX [0x3D] : %d",  (int)(int16_t)raw[9]);
    ESP_LOGI(TAG, "ADY [0x3E] : %d",  (int)(int16_t)raw[10]);
    ESP_LOGI(TAG, "ADZ [0x3F] : %d",  (int)(int16_t)raw[11]);
    ESP_LOGI(TAG, "TEMP[0x40] : %d",  (int)(int16_t)raw[12]);
    ESP_LOGI(TAG, "DX  [0x41] : %d",  (int)(int16_t)raw[13]);
    ESP_LOGI(TAG, "DY  [0x42] : %d",  (int)(int16_t)raw[14]);
    ESP_LOGI(TAG, "DZ  [0x43] : %d",  (int)(int16_t)raw[15]);
    ESP_LOGI(TAG, "HZX [0x44] : %u",  (unsigned)raw[16]);
    ESP_LOGI(TAG, "HZY [0x45] : %u",  (unsigned)raw[17]);
    ESP_LOGI(TAG, "HZZ [0x46] : %u",  (unsigned)raw[18]);
    } else {
    ESP_LOGI(TAG, "TEMP[0x00] : %d",  (int)(int16_t)raw[0]);
    ESP_LOGI(TAG, "VX  [0x01] : %u",  (unsigned)raw[1]);
    ESP_LOGI(TAG, "VY  [0x02] : %u",  (unsigned)raw[2]);
    ESP_LOGI(TAG, "VZ  [0x03] : %u",  (unsigned)raw[3]);
    ESP_LOGI(TAG, "DX  [0x04] : %u",  (unsigned)raw[4]);
    ESP_LOGI(TAG, "DY  [0x05] : %u",  (unsigned)raw[5]);
    ESP_LOGI(TAG, "DZ  [0x06] : %u",  (unsigned)raw[6]);
    ESP_LOGI(TAG, "AX  [0x0A] : %d",  (int)(int16_t)raw[10]);
    ESP_LOGI(TAG, "AY  [0x0B] : %d",  (int)(int16_t)raw[11]);
    ESP_LOGI(TAG, "AZ  [0x0C] : %d",  (int)(int16_t)raw[12]);
    }
}

/**
 * @brief LOG MODE 1 — Print parsed parameters with names and units
 */
static void log_sensor_data(const wtvb01_data_t *d)
{
    ESP_LOGI(TAG, "------- %s Read #%lu -------",
             wtvb01_sensor_model_name(s_model), (unsigned long)d->read_count);

    /* Snapshot accum under mutex to avoid data race with wtvb01_get_stats().
     * s_accum contains 64-bit doubles; non-atomic on 32-bit CPU without the lock. */
    uint32_t n        = 0;
    float vel_avg     = d->velocity_rms;
    float acc_avg     = d->accel_rms;
    float vel_peak    = 0.0f;
    float acc_peak    = 0.0f;
    if (s_mutex != NULL && xSemaphoreTake(s_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        n        = s_accum.sample_count;
        vel_avg  = (n > 0) ? (float)(s_accum.sum_vel_rms   / n) : d->velocity_rms;
        acc_avg  = (n > 0) ? (float)(s_accum.sum_accel_rms / n) : d->accel_rms;
        vel_peak = s_accum.max_vel_rms;
        acc_peak = s_accum.max_accel_rms;
        xSemaphoreGive(s_mutex);
    }

    /* --- LIVEDATA JSON preview (running window stats) --- */
    ESP_LOGI(TAG, "[JSON] vel_rms  now=%.2f mm/s | win_avg=%.2f mm/s | win_peak=%.2f mm/s (%lu samples)",
             d->velocity_rms, vel_avg, vel_peak, (unsigned long)n);
    ESP_LOGI(TAG, "[JSON] acc_rms  now=%.3f g    | win_avg=%.3f g    | win_peak=%.3f g",
             d->accel_rms, acc_avg, acc_peak);
    ESP_LOGI(TAG, "[JSON] vib_temp = %.1f C", d->temperature);

    /* --- XYZ detail for debugging --- */
    ESP_LOGI(TAG, "  Vel  X=%6.2f  Y=%6.2f  Z=%6.2f mm/s",
             d->velocity_x, d->velocity_y, d->velocity_z);
    ESP_LOGI(TAG, "  Acc  X=%6.3f  Y=%6.3f  Z=%6.3f g",
             d->accel_x, d->accel_y, d->accel_z);
    ESP_LOGI(TAG, "  Freq X=%6.1f  Y=%6.1f  Z=%6.1f  DOM=%6.1f Hz",
             d->frequency_x, d->frequency_y, d->frequency_z, d->dominant_frequency);
    ESP_LOGI(TAG, "  Disp X=%6.1f  Y=%6.1f  Z=%6.1f um",
             d->displacement_x, d->displacement_y, d->displacement_z);
    ESP_LOGI(TAG, "  Errors: %lu", (unsigned long)d->error_count);
}

/**
 * @brief LOG MODE 2 — Single CSV line for serial plotter
 *
 * Format (comma-separated, one line per read):
 *   AX,AY,AZ,GX,GY,GZ,VX,VY,VZ,ADX,ADY,ADZ,TEMP,DX,DY,DZ,HX,HY,HZ
 *
 * Prints a header line once on first call so the plotter knows column names.
 */
static void log_plotter_line(const wtvb01_data_t *d)
{
    static bool header_printed = false;
    static wtvb01_sensor_model_t header_model = WTVB01_SENSOR_MODEL_RS_WZ3;
    wtvb01_sensor_model_t model = s_model;
    if (!header_printed || header_model != model) {
        if (s_model == WTVB01_SENSOR_MODEL_RS_WZ3) {
            printf("RS_WZ3:TEMP,VX,VY,VZ,VEL_RMS,DX,DY,DZ,AX,AY,AZ,ACCEL_RMS\n");
        } else {
            printf("WTVB01_485:AX,AY,AZ,GX,GY,GZ,VX,VY,VZ,ADX,ADY,ADZ,TEMP,DX,DY,DZ,HX,HY,HZ\n");
        }
        header_printed = true;
        header_model = model;
    }

    if (s_model == WTVB01_SENSOR_MODEL_RS_WZ3) {
        printf("%.1f,"
               "%.2f,%.2f,%.2f,%.2f,"
               "%.1f,%.1f,%.1f,"
               "%.3f,%.3f,%.3f,%.3f\n",
               d->temperature,
               d->velocity_x, d->velocity_y, d->velocity_z, d->velocity_rms,
               d->displacement_x, d->displacement_y, d->displacement_z,
               d->accel_x, d->accel_y, d->accel_z, d->accel_rms);
    } else {
        printf("%.3f,%.3f,%.3f,"
               "%.2f,%.2f,%.2f,"
               "%.2f,%.2f,%.2f,"
               "%.2f,%.2f,%.2f,"
               "%.2f,"
               "%.1f,%.1f,%.1f,"
               "%.2f,%.2f,%.2f\n",
               d->accel_x, d->accel_y, d->accel_z,
               d->angular_vel_x, d->angular_vel_y, d->angular_vel_z,
               d->velocity_x, d->velocity_y, d->velocity_z,
               d->angle_x, d->angle_y, d->angle_z,
               d->temperature,
               d->displacement_x, d->displacement_y, d->displacement_z,
               d->frequency_x, d->frequency_y, d->frequency_z);
    }
}

/**
 * @brief LOG MODE 3 — Print 1-minute stats summary (livedata fields)
 *
 * Fetches stats via wtvb01_get_stats(reset=true) and prints all fields
 * that would go into a livedata JSON payload.
 */
static void log_stats_summary(void)
{
    wtvb01_stats_t st;
    esp_err_t ret = wtvb01_get_stats(&st, true);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Stats not available yet (%s)", esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(TAG, "==================== LIVEDATA STATS ====================");
    ESP_LOGI(TAG, "Window     : %lu samples, %lu errors",
             (unsigned long)st.sample_count, (unsigned long)st.error_count);
    ESP_LOGI(TAG, "Vel RMS    : avg=%.2f  peak=%.2f mm/s",
             st.velocity_rms_avg, st.velocity_rms_max);
    ESP_LOGI(TAG, "Accel RMS  : avg=%.3f  peak=%.3f g",
             st.accel_rms_avg, st.accel_rms_max);
    ESP_LOGI(TAG, "Disp (avg) : X=%.1f  Y=%.1f  Z=%.1f um",
             st.displacement_x_avg, st.displacement_y_avg, st.displacement_z_avg);
    ESP_LOGI(TAG, "Freq (avg) : X=%.1f  Y=%.1f  Z=%.1f  DOM=%.1f Hz",
             st.frequency_x_avg, st.frequency_y_avg,
             st.frequency_z_avg, st.dominant_freq_avg);
    ESP_LOGI(TAG, "Temperature: %.1f C", st.temperature);
    ESP_LOGI(TAG, "========================================================");
}

/* --------------------------------------------------------------------------
 * Background FreeRTOS task
 * -------------------------------------------------------------------------- */

static void wtvb01_task(void *pvParameters)
{
    (void)pvParameters;

    ESP_LOGI(TAG, "Task started on core %d", (int)xPortGetCoreID());
    esp_task_wdt_add(NULL);

    wtvb01_data_t local = {0};
    TickType_t last_log_tick = xTaskGetTickCount();

    while (s_running) {
        esp_task_wdt_reset();

        uint16_t raw_snap[VIB_MAX_REG_DATA_COUNT];
        memset(raw_snap, 0, sizeof(raw_snap));
        esp_err_t ret = sensor_read_all(&local, raw_snap);

        /* Update shared data structure */
        if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {

            if (ret == ESP_OK) {
                local.data_valid  = true;
                local.read_count  = s_data.read_count + 1;
                local.error_count = s_data.error_count;
                s_data = local;  // only overwrite sensor values on success
            } else {
                // On failure preserve last good sensor values — only update counters
                s_data.error_count = s_data.error_count + 1;
                s_data.data_valid = false;
            }

            /* Accumulate stats inside mutex — s_accum shared with wtvb01_get_stats() */
            if (ret == ESP_OK) {
                stats_accumulate(&local);
            } else {
                s_accum.error_count++;
            }

            xSemaphoreGive(s_mutex);
        } else {
            /* Mutex timeout — successful read result lost; flag it in the log */
            ESP_LOGW(TAG, "Mutex timeout in task loop — read result discarded");
        }

        TickType_t now = xTaskGetTickCount();

        /* --- 1. Statistics Logging (Every 60 Seconds) --- */
#if (WTVB01_LOG_MODE == 3)
        if ((now - last_log_tick) >= pdMS_TO_TICKS(60000)) {
            log_stats_summary();
            last_log_tick = now;
        }
#endif

        /* --- 2. Real-time Logging (Throttled to WTVB01_LOG_INTERVAL_MS, default 1s) --- */
        static TickType_t last_realtime_tick = 0;
        if ((now - last_realtime_tick) >= pdMS_TO_TICKS(WTVB01_LOG_INTERVAL_MS)) {
            if (ret == ESP_OK) {
#if (WTVB01_LOG_MODE == 0)
                log_raw_registers(raw_snap, local.read_count);
#elif (WTVB01_LOG_MODE == 1)
                log_sensor_data(&local);
#elif (WTVB01_LOG_MODE == 2)
                log_plotter_line(&local);
#endif
            } else if (WTVB01_LOG_MODE != 4) {
                ESP_LOGW(TAG, "Read failed (errors: %lu)", (unsigned long)local.error_count);
            }
            last_realtime_tick = now;
        }

        vTaskDelay(pdMS_TO_TICKS(model_poll_interval_ms(s_model)));
    }

    esp_task_wdt_delete(NULL);
    ESP_LOGI(TAG, "Task stopping");
    s_task = NULL;
    vTaskDelete(NULL);
}

/* --------------------------------------------------------------------------
 * Public API
 * -------------------------------------------------------------------------- */

esp_err_t wtvb01_init(void)
{
    if (s_running) {
        ESP_LOGW(TAG, "Already initialised");
        return ESP_OK;
    }

    wtvb01_sensor_model_t model = s_model;
    esp_err_t ret = rs485_bus_init(model_baud_rate(model));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "RS485 bus init failed: %s", esp_err_to_name(ret));
        return ret;
    }

#if 0
    /* Configure UART1 */
    uart_config_t uart_cfg = {
        .baud_rate           = model_baud_rate(model),
        .data_bits           = UART_DATA_8_BITS,
        .parity              = UART_PARITY_DISABLE,
        .stop_bits           = UART_STOP_BITS_1,
        .flow_ctrl           = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk          = UART_SCLK_DEFAULT,
    };

    esp_err_t ret;

    ret = uart_param_config(WTVB01_UART_NUM, &uart_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "uart_param_config failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* TX=GPIO17, RX=GPIO18, RTS=GPIO19 for RS485 DE/RE */
    ret = uart_set_pin(WTVB01_UART_NUM,
                       WTVB01_TX_PIN, WTVB01_RX_PIN,
                       WTVB01_RTS_PIN, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "uart_set_pin failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Install driver with RX buffer only (TX is non-buffered) */
    ret = uart_driver_install(WTVB01_UART_NUM,
                              UART_BUF_SIZE, 0,   /* rx_buf, tx_buf=0 */
                              0, NULL, 0);         /* no event queue */
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "uart_driver_install failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = uart_set_mode(WTVB01_UART_NUM, UART_MODE_RS485_HALF_DUPLEX);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "uart_set_mode RS485 failed: %s", esp_err_to_name(ret));
        uart_driver_delete(WTVB01_UART_NUM);
        return ret;
    }

    ESP_LOGI(TAG,
             "UART1 initialised for %s: %d baud addr 0x%02X poll %lums, "
             "TX=GPIO%d, RX=GPIO%d, RTS=GPIO%d",
             wtvb01_sensor_model_name(model), model_baud_rate(model),
             model_modbus_addr(model),
             (unsigned long)model_poll_interval_ms(model), WTVB01_TX_PIN,
             WTVB01_RX_PIN, WTVB01_RTS_PIN);
#endif
    log_wtvb01_profile_if_needed(model);

    /* Create mutex */
    s_mutex = xSemaphoreCreateMutex();
    if (s_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex");
        rs485_bus_deinit();
        return ESP_FAIL;
    }

    /* Zero-init shared data */
    memset(&s_data, 0, sizeof(s_data));

    /* Start background task pinned to APP_CPU (core 1) */
    s_running = true;
    BaseType_t task_ret = xTaskCreatePinnedToCore(
        wtvb01_task,
        "wtvb01",
        WTVB01_TASK_STACK_SIZE,
        NULL,
        WTVB01_TASK_PRIORITY,
        &s_task,
        APP_CPU_NUM   /* core 1 */
    );

    if (task_ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create task");
        s_running = false;
        vSemaphoreDelete(s_mutex);
        s_mutex = NULL;
        rs485_bus_deinit();
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Component initialised successfully");
    return ESP_OK;
}

esp_err_t wtvb01_deinit(void)
{
    if (!s_running) {
        return ESP_OK;
    }

    /* Signal task to stop; it deletes itself */
    s_running = false;

    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(1000);
    while (s_task != NULL && xTaskGetTickCount() < deadline) {
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    if (s_task != NULL) {
        ESP_LOGW(TAG, "Task did not confirm stop before deinit timeout");
    }

    if (s_mutex != NULL) {
        vSemaphoreDelete(s_mutex);
        s_mutex = NULL;
    }

    rs485_bus_deinit();

    ESP_LOGI(TAG, "Component de-initialised");
    return ESP_OK;
}

esp_err_t wtvb01_read_all(wtvb01_data_t *data)
{
    if (data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_running || s_mutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(50)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    *data = s_data;
    xSemaphoreGive(s_mutex);

    return ESP_OK;
}

esp_err_t wtvb01_read_velocity(float *vx, float *vy, float *vz)
{
    if (!s_running || s_mutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(50)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    bool valid = s_data.data_valid;
    if (vx) *vx = s_data.velocity_x;
    if (vy) *vy = s_data.velocity_y;
    if (vz) *vz = s_data.velocity_z;
    xSemaphoreGive(s_mutex);

    return valid ? ESP_OK : ESP_ERR_INVALID_STATE;
}

esp_err_t wtvb01_read_temperature(float *temp_c)
{
    if (temp_c == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_running || s_mutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(50)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    bool valid = s_data.data_valid;
    *temp_c = s_data.temperature;
    xSemaphoreGive(s_mutex);

    return valid ? ESP_OK : ESP_ERR_INVALID_STATE;
}

esp_err_t wtvb01_get_stats(wtvb01_stats_t *stats, bool reset)
{
    if (stats == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_running || s_mutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    uint32_t n = s_accum.sample_count;

    if (n == 0) {
        /* No samples collected yet */
        memset(stats, 0, sizeof(*stats));
        stats->error_count = s_accum.error_count;
        xSemaphoreGive(s_mutex);
        return ESP_ERR_INVALID_STATE;
    }

    float inv_n = 1.0f / (float)n;

    /* Compute averages */
    stats->velocity_rms_avg    = (float)(s_accum.sum_vel_rms   * inv_n);
    stats->velocity_rms_max    = s_accum.max_vel_rms;

    stats->accel_rms_avg       = (float)(s_accum.sum_accel_rms * inv_n);
    stats->accel_rms_max       = s_accum.max_accel_rms;

    stats->displacement_x_avg  = (float)(s_accum.sum_disp_x    * inv_n);
    stats->displacement_y_avg  = (float)(s_accum.sum_disp_y    * inv_n);
    stats->displacement_z_avg  = (float)(s_accum.sum_disp_z    * inv_n);

    stats->frequency_x_avg     = (float)(s_accum.sum_freq_x    * inv_n);
    stats->frequency_y_avg     = (float)(s_accum.sum_freq_y    * inv_n);
    stats->frequency_z_avg     = (float)(s_accum.sum_freq_z    * inv_n);
    stats->dominant_freq_avg   = (float)(s_accum.sum_dom_freq  * inv_n);

    stats->temperature         = s_accum.last_temp;

    stats->sample_count        = n;
    stats->error_count         = s_accum.error_count;

    if (reset) {
        stats_reset();
    }

    xSemaphoreGive(s_mutex);
    return ESP_OK;
}

// ============================================================
// CALIBRATION FUNCTIONS - Write to sensor registers
// ============================================================

// Register addresses for calibration values (from RS-WZ3 datasheet)
#define REG_X_FACTOR  0x0060  // X-axis speed coefficient A (factor)
#define REG_X_OFFSET  0x0062  // X-axis speed coefficient B (offset)
#define REG_Y_FACTOR  0x0064  // Y-axis speed coefficient A (factor)
#define REG_Y_OFFSET  0x0066  // Y-axis speed coefficient B (offset)
#define REG_Z_FACTOR  0x0068  // Z-axis speed coefficient A (factor)
#define REG_Z_OFFSET  0x006A  // Z-axis speed coefficient B (offset)

/**
 * @brief Get register address for calibration parameter
 * @param axis 'X', 'Y', or 'Z'
 * @param is_offset true for offset (B), false for factor (A)
 * @return Register address, or 0xFFFF if invalid axis
 */
static uint16_t get_calibration_register(char axis, bool is_offset) {
    switch (axis) {
        case 'X':
        case 'x':
            return is_offset ? REG_X_OFFSET : REG_X_FACTOR;
        case 'Y':
        case 'y':
            return is_offset ? REG_Y_OFFSET : REG_Y_FACTOR;
        case 'Z':
        case 'z':
            return is_offset ? REG_Z_OFFSET : REG_Z_FACTOR;
        default:
            return 0xFFFF;
    }
}

/**
 * @brief Convert float to two uint16_t registers (32-bit IEEE 754 big-endian)
 */
static void float_to_registers(float value, uint16_t *reg_high, uint16_t *reg_low) {
    union {
        float f;
        uint32_t u;
    } converter = {.f = value};
    uint32_t bits = converter.u;
    *reg_high = (uint16_t)(bits >> 16);
    *reg_low = (uint16_t)(bits & 0xFFFF);
}

/**
 * @brief Convert two uint16_t registers to float (32-bit IEEE 754 big-endian)
 */
static float registers_to_float(uint16_t reg_high, uint16_t reg_low) {
    union {
        uint32_t u;
        float f;
    } converter;
    converter.u = ((uint32_t)reg_high << 16) | (uint32_t)reg_low;
    return converter.f;
}

/**
 * @brief Send Modbus write single register command (FC 0x06)
 * 
 * @param register_addr Modbus register address
 * @param value Value to write
 * @return ESP_OK on success
 */
static esp_err_t wtvb01_write_register(uint16_t register_addr, uint16_t value) {
    if (!s_mutex) {
        ESP_LOGE(TAG, "wtvb01 not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);

    // Build Modbus FC 0x06 (Write Single Register) command
    // Frame: [addr(1)] [func(1)] [reg_hi(1)] [reg_lo(1)] [val_hi(1)] [val_lo(1)] [crc_lo(1)] [crc_hi(1)]
    uint8_t cmd[8];
    cmd[0] = WTVB01_MODBUS_ADDR;  // Slave address
    cmd[1] = 0x06;                 // Function code: Write Single Register
    cmd[2] = (uint8_t)(register_addr >> 8);     // Register address high byte
    cmd[3] = (uint8_t)(register_addr & 0xFF);   // Register address low byte
    cmd[4] = (uint8_t)(value >> 8);             // Value high byte
    cmd[5] = (uint8_t)(value & 0xFF);           // Value low byte

    // Add CRC
    modbus_add_crc(cmd, 6);

    uint8_t response[16] = {0};
    int rx_bytes = 0;
    esp_err_t bus_ret = rs485_bus_transact(
        model_baud_rate(s_model), cmd, 8, response, sizeof(response),
        WTVB01_RESPONSE_TIMEOUT_MS, PRE_TX_DELAY_US, POST_TX_DELAY_US,
        model_inter_frame_delay_us(s_model), &rx_bytes);

    xSemaphoreGive(s_mutex);

    if (bus_ret != ESP_OK) {
        ESP_LOGE(TAG, "RS485 write transaction failed: %s",
                 esp_err_to_name(bus_ret));
        return bus_ret;
    }

    if (rx_bytes < 8) {
        ESP_LOGE(TAG, "No response to write register (got %d bytes)", rx_bytes);
        return ESP_ERR_TIMEOUT;
    }

    // Verify response (echoes the request for FC 0x06)
    if (rx_bytes >= 8 && 
        response[0] == WTVB01_MODBUS_ADDR && 
        response[1] == 0x06 &&
        modbus_check_crc(response, 8)) {
        ESP_LOGI(TAG, "Register 0x%04X written successfully with value 0x%04X", 
                 register_addr, value);
        return ESP_OK;
    }

    ESP_LOGE(TAG, "Invalid write response (CRC or format error)");
    return ESP_FAIL;
}

esp_err_t wtvb01_write_calibration_factor(char axis, float factor) {
    if (s_model != WTVB01_SENSOR_MODEL_RS_WZ3) {
        ESP_LOGW(TAG, "Calibration registers are only supported for RS-WZ3");
        return ESP_ERR_NOT_SUPPORTED;
    }

    // Register address for factor (A) based on axis
    uint16_t reg_addr = get_calibration_register(axis, false);
    if (reg_addr == 0xFFFF) {
        ESP_LOGE(TAG, "Invalid axis: %c (use X, Y, or Z)", axis);
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "Writing %c-axis calibration factor: %.6f", axis, factor);
    
    // Factors are typically stored as floats (2 registers each)
    // For simplicity, we'll write the high byte using FC 0x06
    uint16_t reg_high, reg_low;
    float_to_registers(factor, &reg_high, &reg_low);
    
    // Write high word first
    esp_err_t ret = wtvb01_write_register(reg_addr, reg_high);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Write low word
    return wtvb01_write_register(reg_addr + 1, reg_low);
}

esp_err_t wtvb01_write_calibration_offset(char axis, float offset) {
    if (s_model != WTVB01_SENSOR_MODEL_RS_WZ3) {
        ESP_LOGW(TAG, "Calibration registers are only supported for RS-WZ3");
        return ESP_ERR_NOT_SUPPORTED;
    }

    // Register address for offset (B) based on axis
    uint16_t reg_addr = get_calibration_register(axis, true);
    if (reg_addr == 0xFFFF) {
        ESP_LOGE(TAG, "Invalid axis: %c (use X, Y, or Z)", axis);
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "Writing %c-axis calibration offset: %.6f", axis, offset);
    
    // Offsets are typically stored as floats (2 registers each)
    uint16_t reg_high, reg_low;
    float_to_registers(offset, &reg_high, &reg_low);
    
    // Write high word first
    esp_err_t ret = wtvb01_write_register(reg_addr, reg_high);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Write low word
    return wtvb01_write_register(reg_addr + 1, reg_low);
}

esp_err_t wtvb01_read_calibration_all(float *x_factor, float *x_offset,
                                       float *y_factor, float *y_offset,
                                       float *z_factor, float *z_offset) {
    if (s_model != WTVB01_SENSOR_MODEL_RS_WZ3) {
        ESP_LOGW(TAG, "Calibration registers are only supported for RS-WZ3");
        return ESP_ERR_NOT_SUPPORTED;
    }

    if (!s_mutex) {
        return ESP_ERR_INVALID_STATE;
    }

    if (!x_factor && !x_offset && !y_factor && !y_offset && !z_factor && !z_offset) {
        return ESP_ERR_INVALID_ARG;
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);

    // Build read command for all calibration registers (12 registers total)
    // 0x0060-0x006B (6 pairs)
    uint8_t cmd[8];
    modbus_build_read_cmd(cmd, REG_X_FACTOR, 12);

    uint8_t response[50] = {0};  // Need larger buffer for 12 registers
    int rx_bytes = 0;
    esp_err_t bus_ret = rs485_bus_transact(
        model_baud_rate(s_model), cmd, 8, response, sizeof(response),
        WTVB01_RESPONSE_TIMEOUT_MS, PRE_TX_DELAY_US, POST_TX_DELAY_US,
        model_inter_frame_delay_us(s_model), &rx_bytes);

    xSemaphoreGive(s_mutex);

    if (bus_ret != ESP_OK) {
        ESP_LOGE(TAG, "RS485 calibration read failed: %s",
                 esp_err_to_name(bus_ret));
        return bus_ret;
    }

    if (rx_bytes < 29) {  // 5 + 12*2 = 29 bytes minimum
        ESP_LOGE(TAG, "Failed to read calibration (got %d bytes)", rx_bytes);
        return ESP_ERR_TIMEOUT;
    }

    uint16_t regs[12] = {0};
    int parsed = modbus_parse_response(response, rx_bytes, regs, 12);

    if (parsed != 12) {
        ESP_LOGE(TAG, "Failed to parse calibration response");
        return ESP_FAIL;
    }

    // Convert register pairs to floats
    if (x_factor) *x_factor = registers_to_float(regs[0], regs[1]);
    if (x_offset) *x_offset = registers_to_float(regs[2], regs[3]);
    if (y_factor) *y_factor = registers_to_float(regs[4], regs[5]);
    if (y_offset) *y_offset = registers_to_float(regs[6], regs[7]);
    if (z_factor) *z_factor = registers_to_float(regs[8], regs[9]);
    if (z_offset) *z_offset = registers_to_float(regs[10], regs[11]);

    ESP_LOGI(TAG, "Calibration read: X(%.4f/%.4f) Y(%.4f/%.4f) Z(%.4f/%.4f)",
             x_factor ? *x_factor : 0.0f, x_offset ? *x_offset : 0.0f,
             y_factor ? *y_factor : 0.0f, y_offset ? *y_offset : 0.0f,
             z_factor ? *z_factor : 0.0f, z_offset ? *z_offset : 0.0f);
    
    return ESP_OK;
}
