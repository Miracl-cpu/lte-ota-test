/**
 * @file daq_ct_ads1115.c
 * @brief CT Current Data Acquisition using ADS1115 ADC
 *
 * ADS1115 Configuration:
 * - PGA: +/-4.096V (LSB = 125uV)
 * - Data Rate: 860 SPS (~1.16ms/conversion) — required for 50 Hz CT signals
 *   NOTE: rates below ~250 SPS heavily attenuate 50 Hz via the internal sinc³
 *   filter (16 SPS gives ~96% attenuation → 20× current underread).
 * - Mode: Single-shot
 * - MUX: Single-ended AIN0, AIN1, AIN2
 * - ALERT/RDY: Interrupt-driven conversion-ready via GPIO (falling edge)
 *
 * CT Processing:
 * - Unipolar waveform (0-1V, NOT mid-biased)
 * - RMS = sqrt(sum(v^2)/n)
 * - Current = Vrms * 100 A/V * calibration factor
 * - Open CT detection: high voltage + low RMS
 *
 * Mutex split design:
 * - Mutex held ~100us to start conversion, released during ~1.16ms conversion wait
 * - Mutex held ~100us to read result — LCD/keypad never blocked during conversion
 */

#include "daq_ct_ads1115.h"
#include "driver/gpio.h"
#include "esp_task_wdt.h"
#include "log_config.h"
#undef LOG_LOCAL_LEVEL
#define LOG_LOCAL_LEVEL LOG_LEVEL_DAQ_CT
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <string.h>
#include <math.h>

static const char *TAG = "daq_ct";

// ADS1115 Register addresses
#define ADS1115_REG_CONVERSION  0x00
#define ADS1115_REG_CONFIG      0x01
#define ADS1115_REG_LO_THRESH   0x02
#define ADS1115_REG_HI_THRESH   0x03

// ADS1115 Config register bits
#define ADS1115_OS_START        (1 << 15)  // Start single conversion
#define ADS1115_OS_BUSY         (1 << 15)  // Conversion in progress (read)

// MUX settings (bits 14:12)
#define ADS1115_MUX_AIN0_GND    (0x04 << 12)  // AIN0 vs GND
#define ADS1115_MUX_AIN1_GND    (0x05 << 12)  // AIN1 vs GND
#define ADS1115_MUX_AIN2_GND    (0x06 << 12)  // AIN2 vs GND
#define ADS1115_MUX_AIN3_GND    (0x07 << 12)  // AIN3 vs GND

// PGA settings (bits 11:9)
// ±2.048V chosen: CT signal ~100mV peak fits well; open-CT pullup (~3V) saturates
// ADC to +32767 → raw_to_voltage = 2.048V > CT_OPEN_VRMS_THRESH (1.5V) ✓
// Resolution: 62.5 µV/LSB — 4× better than ±4.096V.
#define ADS1115_PGA_2048        (0x02 << 9)   // +/-2.048V

// Mode (bit 8)
#define ADS1115_MODE_SINGLE     (1 << 8)      // Single-shot mode

// Data rate (bits 7:5)
// 860 SPS (~1.16ms/conv) is required — the internal sinc³ filter at slower rates
// attenuates 50 Hz CT signals severely (16 SPS → ~96% attenuation → 20× underread).
#define ADS1115_DR_860          (0x07 << 5)   // 860 SPS — ~1.16 ms/conversion

// Comparator settings
#define ADS1115_COMP_LAT        (1 << 2)      // Latching comparator (ALERT stays low until conv reg read)
#define ADS1115_COMP_QUE_1      0x00          // Assert after 1 conversion

// Combined config: 860 SPS, latching comparator, assert after 1 conversion
#define ADS1115_CONFIG_BASE     (ADS1115_PGA_2048 | ADS1115_MODE_SINGLE | \
                                 ADS1115_DR_860 | ADS1115_COMP_LAT | ADS1115_COMP_QUE_1)

// Conversion factor: LSB = 2.048V / 32768 = 62.5uV
#define ADS1115_LSB_MV          0.0625f
#define ADS1115_LSB_V           0.0000625f

// DAQ parameters
// 50 samples × 3 phases × ~1.16ms ≈ 174ms per RMS window
#define CT_WINDOW_SAMPLES       50
// CT_AMPS_PER_VRMS is now stored in daq_ct_handle_s.ct_amps_per_vrms (configurable via daq_config_t)
// CT Open Detection:
// - Disconnected CT floats HIGH (near rail voltage ~3V)
// - Connected CT shows burden resistor output (< 1.5V when measuring)
#define CT_OPEN_VRMS_THRESH     1.5f    // CT is OPEN if Vrms >= this (1.5V)
#define CT_OPEN_DEBOUNCE        3       // Consecutive windows for open detect

// Phase MUX settings
static const uint16_t PHASE_MUX[3] = {
    ADS1115_MUX_AIN0_GND,  // Phase A
    ADS1115_MUX_AIN1_GND,  // Phase B
    ADS1115_MUX_AIN2_GND   // Phase C
};

/**
 * @brief Internal DAQ handle structure
 */
struct daq_ct_handle_s {
    i2c_mux_mgr_t *mux_mgr;
    i2c_master_dev_handle_t ads_handle;
    uint8_t ads_addr;
    uint8_t tca_channel;
    float cal_factor[3];
    float ct_amps_per_vrms;     // CT ratio: Amps per Vrms (configurable, default 100.0)

    // Thread safety
    SemaphoreHandle_t snapshot_mutex;

    // ALERT/RDY interrupt-driven conversion
    SemaphoreHandle_t conv_ready_sem;  // Given by ALERT ISR when conversion complete
    SemaphoreHandle_t task_done_sem;   // Given by task just before vTaskDelete
    int alert_gpio;                    // GPIO pin for ALERT/RDY (-1 = polling mode)
    volatile uint32_t isr_fire_count;  // Debug: total ISR firings

    // Latest snapshot
    daq_snapshot_t snapshot;

    // Accumulation buffers
    float v_sum_sq[3];          // Sum of v^2
    float v_sum[3];             // Sum of v
    float v_max[3];             // Max voltage in window
    uint32_t sample_count;      // Outer-loop iterations with ≥1 successful phase
    uint32_t phase_sample_count[3]; // Successful samples per phase (divisor for RMS)

    // CT open debounce
    uint8_t ct_open_count[3];   // Consecutive open detections
    uint8_t ct_close_count[3];  // Consecutive close detections

    // Error tracking
    uint32_t i2c_err_count;
    uint32_t ads_err_count;
    int64_t last_err_log_time;

    // Running flag
    bool running;
};

// Global handle for easy access
static daq_ct_handle_t *g_daq_handle = NULL;

// Context for ADS read operations
typedef struct {
    daq_ct_handle_t *daq;
    uint8_t phase;
    int16_t raw_value;
    esp_err_t result;
} ads_read_ctx_t;

/**
 * @brief ISR handler for ADS1115 ALERT/RDY pin (falling edge = conversion complete)
 */
static void IRAM_ATTR ads_alert_isr_handler(void *arg)
{
    daq_ct_handle_t *daq = (daq_ct_handle_t *)arg;
    daq->isr_fire_count++;
    BaseType_t woken = pdFALSE;
    xSemaphoreGiveFromISR(daq->conv_ready_sem, &woken);
    portYIELD_FROM_ISR(woken);
}

/**
 * @brief Write ADS1115 config register (within mux context)
 */
static esp_err_t ads_write_config(daq_ct_handle_t *daq, uint16_t config)
{
    uint8_t data[3];
    data[0] = ADS1115_REG_CONFIG;
    data[1] = (config >> 8) & 0xFF;    // MSB
    data[2] = config & 0xFF;           // LSB

    return i2c_master_transmit(daq->ads_handle, data, 3, pdMS_TO_TICKS(10));
}

/**
 * @brief Read ADS1115 conversion register (within mux context)
 */
static esp_err_t ads_read_conversion(daq_ct_handle_t *daq, int16_t *value)
{
    uint8_t reg = ADS1115_REG_CONVERSION;
    uint8_t data[2];

    esp_err_t ret = i2c_master_transmit_receive(daq->ads_handle, &reg, 1,
                                                  data, 2, pdMS_TO_TICKS(10));
    if (ret == ESP_OK) {
        *value = ((int16_t)data[0] << 8) | data[1];
    }
    return ret;
}

/**
 * @brief Callback to configure ALERT/RDY threshold registers (called once at init)
 *
 * Sets Lo_thresh=0x0000 and Hi_thresh=0x8000 to enable ALERT/RDY pin mode.
 * Must be called within an i2c_mux_mgr_exec context.
 */
static esp_err_t ads_config_alert_rdy_callback(void *ctx)
{
    daq_ct_handle_t *daq = (daq_ct_handle_t *)ctx;
    esp_err_t ret;

    // Lo_thresh = 0x0000 (MSB=0 enables ALERT/RDY mode)
    uint8_t lo_data[3] = {ADS1115_REG_LO_THRESH, 0x00, 0x00};
    ret = i2c_master_transmit(daq->ads_handle, lo_data, 3, pdMS_TO_TICKS(10));
    if (ret != ESP_OK) return ret;

    // Hi_thresh = 0x8000 (MSB=1 enables ALERT/RDY mode)
    uint8_t hi_data[3] = {ADS1115_REG_HI_THRESH, 0x80, 0x00};
    return i2c_master_transmit(daq->ads_handle, hi_data, 3, pdMS_TO_TICKS(10));
}

/**
 * @brief Callback 1: Start a single-shot conversion (within mux context)
 *
 * Drains any stale semaphore token, then writes config to trigger conversion.
 * Mutex is released immediately after; the ~1.16ms conversion happens asynchronously.
 */
static esp_err_t ads_start_conversion_callback(void *ctx)
{
    ads_read_ctx_t *rctx = (ads_read_ctx_t *)ctx;
    daq_ct_handle_t *daq = rctx->daq;

    // Drain any stale software semaphore token
    xSemaphoreTake(daq->conv_ready_sem, 0);

    // Clear any stale hardware ALERT latch (COMP_LAT=1).
    // If the previous ads_read_result_callback failed (I2C error), the ADS1115
    // conversion register was never read, so ALERT remains asserted LOW indefinitely.
    // Reading the conversion register here deasserts the latch so the next completed
    // conversion produces a clean falling edge for the GPIO ISR.
    // Return value ignored — this is best-effort latch housekeeping.
    int16_t _latch_clr;
    ads_read_conversion(daq, &_latch_clr);

    uint16_t config = ADS1115_OS_START | PHASE_MUX[rctx->phase] | ADS1115_CONFIG_BASE;
    ESP_LOGV(TAG, "  [ph%d] start conv: config=0x%04X (isr_total=%lu)",
             rctx->phase, config, (unsigned long)daq->isr_fire_count);
    esp_err_t ret = ads_write_config(daq, config);
    if (ret != ESP_OK) {
        daq->i2c_err_count++;
    }
    rctx->result = ret;
    return ret;
}

/**
 * @brief Callback 2: Read conversion result (within mux context)
 *
 * Called after ALERT/RDY signals conversion complete.
 * Reading the conversion register also clears the latched ALERT pin.
 */
static esp_err_t ads_read_result_callback(void *ctx)
{
    ads_read_ctx_t *rctx = (ads_read_ctx_t *)ctx;
    daq_ct_handle_t *daq = rctx->daq;

    esp_err_t ret = ads_read_conversion(daq, &rctx->raw_value);
    if (ret != ESP_OK) {
        daq->i2c_err_count++;
    }
    rctx->result = ret;
    return ret;
}

/**
 * @brief Convert raw ADC value to voltage
 */
static float raw_to_voltage(int16_t raw)
{
    return (float)raw * ADS1115_LSB_V;
}

/**
 * @brief Process accumulated samples and compute RMS
 *
 * All math (sqrtf, debounce logic) is done in local variables BEFORE the mutex.
 * The snapshot_mutex is held only for the final struct copy (~1 µs), so readers
 * (daq_ct_get_latest) are never blocked for more than a trivial memcpy.
 */
static void process_window(daq_ct_handle_t *daq)
{
    if (daq->sample_count == 0) return;

    int64_t now_ms = esp_timer_get_time() / 1000;

    // --- Phase 0: Snapshot cal_factor under mutex (~1 µs) so that
    //     daq_ct_update_cal_factors() can safely write cal_factor[] at any time. ---
    float local_cal[3] = {1.0f, 1.0f, 1.0f};  // safe fallback if mutex times out
    if (xSemaphoreTake(daq->snapshot_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        local_cal[0] = daq->cal_factor[0];
        local_cal[1] = daq->cal_factor[1];
        local_cal[2] = daq->cal_factor[2];
        xSemaphoreGive(daq->snapshot_mutex);
    } else {
        ESP_LOGW(TAG, "process_window: cal snapshot mutex timeout — using unity cal this cycle");
    }

    // --- Phase 1: Compute all results in local variables (no mutex needed) ---

    float lv_rms[3]  = {0};
    float lv_mean[3] = {0};
    float li_rms[3]  = {0};
    bool  phase_computed[3] = {false};

    // Preserve current open mask; only update bits for phases that have samples
    uint8_t new_ct_open_mask = daq->snapshot.ct_open_mask;

    for (int ph = 0; ph < 3; ph++) {
        // Skip phases with no successful samples this window (I2C error on that phase).
        // Leaves the snapshot value and CT-open debounce unchanged rather than
        // reporting 0 A and triggering a false CT-open state.
        if (daq->phase_sample_count[ph] == 0) continue;
        phase_computed[ph] = true;

        float n = (float)daq->phase_sample_count[ph];  // per-phase divisor

        // RMS = sqrt(sum(v^2)/n),  mean = sum(v)/n
        lv_rms[ph]  = sqrtf(daq->v_sum_sq[ph] / n);
        lv_mean[ph] = daq->v_sum[ph] / n;

        // Irms = Vrms × ct_amps_per_vrms × cal_factor (local snapshot from Phase 0)
        li_rms[ph] = lv_rms[ph] * daq->ct_amps_per_vrms * local_cal[ph];

        // CT open detection: open CT floats near rail; Vrms >= threshold → open
        bool ct_open_detected = (lv_rms[ph] >= CT_OPEN_VRMS_THRESH);

        if (ct_open_detected) {
            daq->ct_open_count[ph]++;
            daq->ct_close_count[ph] = 0;
            if (daq->ct_open_count[ph] >= CT_OPEN_DEBOUNCE) {
                new_ct_open_mask |= (1 << ph);
                daq->ct_open_count[ph] = CT_OPEN_DEBOUNCE;  // Cap
            }
        } else {
            daq->ct_close_count[ph]++;
            daq->ct_open_count[ph] = 0;
            if (daq->ct_close_count[ph] >= CT_OPEN_DEBOUNCE) {
                new_ct_open_mask &= ~(uint8_t)(1 << ph);
                daq->ct_close_count[ph] = CT_OPEN_DEBOUNCE;  // Cap
            }
        }
    }

    // --- Phase 2: Hold mutex only for the final snapshot copy (~1 µs) ---
    if (xSemaphoreTake(daq->snapshot_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        daq->snapshot.seq++;
        daq->snapshot.ts_ms         = now_ms;
        daq->snapshot.i2c_err_count = daq->i2c_err_count;
        daq->snapshot.ads_err_count = daq->ads_err_count;
        daq->snapshot.ct_open_mask  = new_ct_open_mask;

        if (phase_computed[0]) { daq->snapshot.va_rms = lv_rms[0];  daq->snapshot.va_mean = lv_mean[0];  daq->snapshot.ia_rms = li_rms[0]; }
        if (phase_computed[1]) { daq->snapshot.vb_rms = lv_rms[1];  daq->snapshot.vb_mean = lv_mean[1];  daq->snapshot.ib_rms = li_rms[1]; }
        if (phase_computed[2]) { daq->snapshot.vc_rms = lv_rms[2];  daq->snapshot.vc_mean = lv_mean[2];  daq->snapshot.ic_rms = li_rms[2]; }

        xSemaphoreGive(daq->snapshot_mutex);
    } else {
        ESP_LOGW(TAG, "process_window: snapshot mutex timeout — skipping snapshot update this cycle");
    }

    // --- Phase 3: Log (outside mutex, uses local arrays) ---
    static int64_t last_log = 0;
    if (now_ms - last_log > 2000) {
        last_log = now_ms;
        ESP_LOGI(TAG, "=== CT Readings (open_mask=0x%02X  isr_total=%lu  i2c_err=%lu  ads_err=%lu) ===",
                 new_ct_open_mask, (unsigned long)daq->isr_fire_count,
                 (unsigned long)daq->i2c_err_count, (unsigned long)daq->ads_err_count);
        ESP_LOGI(TAG, "  A: I=%.3fA  Vrms=%.4fV  Vmean=%.4fV  Vmax=%.4fV  (open=%d)",
                 li_rms[0], lv_rms[0], lv_mean[0], daq->v_max[0], (new_ct_open_mask & 0x01) ? 1 : 0);
        ESP_LOGI(TAG, "  B: I=%.3fA  Vrms=%.4fV  Vmean=%.4fV  Vmax=%.4fV  (open=%d)",
                 li_rms[1], lv_rms[1], lv_mean[1], daq->v_max[1], (new_ct_open_mask & 0x02) ? 1 : 0);
        ESP_LOGI(TAG, "  C: I=%.3fA  Vrms=%.4fV  Vmean=%.4fV  Vmax=%.4fV  (open=%d)",
                 li_rms[2], lv_rms[2], lv_mean[2], daq->v_max[2], (new_ct_open_mask & 0x04) ? 1 : 0);
        ESP_LOGI(TAG, "  Open detection: Vrms >= %.4fV (ratio=%.1fA/V)", CT_OPEN_VRMS_THRESH, daq->ct_amps_per_vrms);
    }

    // Reset accumulators
    memset(daq->v_sum_sq, 0, sizeof(daq->v_sum_sq));
    memset(daq->v_sum, 0, sizeof(daq->v_sum));
    memset(daq->v_max, 0, sizeof(daq->v_max));
    memset(daq->phase_sample_count, 0, sizeof(daq->phase_sample_count));
    daq->sample_count = 0;
}

esp_err_t daq_ct_init(const daq_config_t *config, daq_ct_handle_t **handle)
{
    if (config == NULL || handle == NULL || config->mux_mgr == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    daq_ct_handle_t *daq = calloc(1, sizeof(daq_ct_handle_t));
    if (daq == NULL) {
        return ESP_ERR_NO_MEM;
    }

    daq->mux_mgr = config->mux_mgr;
    daq->ads_addr = config->ads_addr;
    daq->tca_channel = config->tca_channel;
    daq->alert_gpio = config->alert_gpio;

    // Copy calibration factors
    for (int i = 0; i < 3; i++) {
        daq->cal_factor[i] = (config->cal_factor[i] > 0) ? config->cal_factor[i] : 1.0f;
    }
    daq->ct_amps_per_vrms = (config->ct_amps_per_vrms > 0) ? config->ct_amps_per_vrms : 100.0f;

    // Create snapshot mutex
    daq->snapshot_mutex = xSemaphoreCreateMutex();
    if (daq->snapshot_mutex == NULL) {
        free(daq);
        return ESP_ERR_NO_MEM;
    }

    // Create binary semaphore for ALERT/RDY signalling
    daq->conv_ready_sem = xSemaphoreCreateBinary();
    if (daq->conv_ready_sem == NULL) {
        vSemaphoreDelete(daq->snapshot_mutex);
        free(daq);
        return ESP_ERR_NO_MEM;
    }

    // Create binary semaphore for task-exit handshake (deinit waits on this)
    daq->task_done_sem = xSemaphoreCreateBinary();
    if (daq->task_done_sem == NULL) {
        vSemaphoreDelete(daq->conv_ready_sem);
        vSemaphoreDelete(daq->snapshot_mutex);
        free(daq);
        return ESP_ERR_NO_MEM;
    }

    // Add ADS1115 device to I2C bus
    esp_err_t ret = i2c_mux_mgr_add_device(config->mux_mgr, config->ads_addr, &daq->ads_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add ADS1115 @0x%02X: %s", config->ads_addr, esp_err_to_name(ret));
        vSemaphoreDelete(daq->task_done_sem);
        vSemaphoreDelete(daq->conv_ready_sem);
        vSemaphoreDelete(daq->snapshot_mutex);
        free(daq);
        return ret;
    }

    if (config->alert_gpio >= 0) {
        // Interrupt mode: configure GPIO and install ALERT/RDY ISR

        gpio_config_t io_conf = {
            .pin_bit_mask = (1ULL << config->alert_gpio),
            .mode         = GPIO_MODE_INPUT,
            .pull_up_en   = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type    = GPIO_INTR_NEGEDGE,   // ALERT is active-low open-drain
        };
        ret = gpio_config(&io_conf);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "GPIO%d config failed: %s", config->alert_gpio, esp_err_to_name(ret));
            vSemaphoreDelete(daq->task_done_sem);
            vSemaphoreDelete(daq->conv_ready_sem);
            vSemaphoreDelete(daq->snapshot_mutex);
            free(daq);
            return ret;
        }

        // Install GPIO ISR service (ESP_ERR_INVALID_STATE = already installed by another driver, OK)
        esp_err_t isr_ret = gpio_install_isr_service(0);
        if (isr_ret != ESP_OK && isr_ret != ESP_ERR_INVALID_STATE) {
            ESP_LOGE(TAG, "ISR service install failed: %s", esp_err_to_name(isr_ret));
            vSemaphoreDelete(daq->task_done_sem);
            vSemaphoreDelete(daq->conv_ready_sem);
            vSemaphoreDelete(daq->snapshot_mutex);
            free(daq);
            return isr_ret;
        }

        ret = gpio_isr_handler_add(config->alert_gpio, ads_alert_isr_handler, daq);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "ISR handler add failed: %s", esp_err_to_name(ret));
            vSemaphoreDelete(daq->task_done_sem);
            vSemaphoreDelete(daq->conv_ready_sem);
            vSemaphoreDelete(daq->snapshot_mutex);
            free(daq);
            return ret;
        }

        // Write Lo_thresh=0x0000 and Hi_thresh=0x8000 to enable ALERT/RDY pin mode
        ret = i2c_mux_mgr_exec(config->mux_mgr, config->tca_channel,
                                ads_config_alert_rdy_callback, daq);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "ALERT/RDY threshold config failed: %s", esp_err_to_name(ret));
            gpio_isr_handler_remove(config->alert_gpio);
            vSemaphoreDelete(daq->task_done_sem);
            vSemaphoreDelete(daq->conv_ready_sem);
            vSemaphoreDelete(daq->snapshot_mutex);
            free(daq);
            return ret;
        }

        ESP_LOGI(TAG, "DAQ initialized: ADS@0x%02X on TCA CH%d, interrupt on GPIO%d",
                 config->ads_addr, config->tca_channel, config->alert_gpio);
    } else {
        // Polling mode: no GPIO — task waits a fixed 2ms (> 1.16ms at 860 SPS)
        ESP_LOGI(TAG, "DAQ initialized: ADS@0x%02X on TCA CH%d, polling mode (no ALERT/RDY GPIO)",
                 config->ads_addr, config->tca_channel);
    }

    daq->running = true;
    g_daq_handle = daq;
    *handle = daq;

    return ESP_OK;
}

void daq_ct_task(void *arg)
{
    daq_ct_handle_t *daq = (daq_ct_handle_t *)arg;

    if (daq == NULL) {
        ESP_LOGE(TAG, "DAQ task started with NULL handle");
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "DAQ task started");

    // Subscribe this task to the Task Watchdog Timer
    esp_task_wdt_add(NULL);
    ESP_LOGI(TAG, "DAQ task subscribed to TWDT");

    while (daq->running) {
        // Feed watchdog every loop iteration (~4ms at 860 SPS)
        esp_task_wdt_reset();

        bool phase_ok = false;
        for (uint8_t phase = 0; phase < 3; phase++) {
            ads_read_ctx_t ctx = {
                .daq       = daq,
                .phase     = phase,
                .raw_value = 0,
                .result    = ESP_OK
            };

            // Step 1: Start conversion (~100 µs mutex hold)
            esp_err_t ret = i2c_mux_mgr_exec(daq->mux_mgr, daq->tca_channel,
                                              ads_start_conversion_callback, &ctx);
            if (ret != ESP_OK || ctx.result != ESP_OK) {
                int64_t now = esp_timer_get_time();
                if (now - daq->last_err_log_time > 5000000) {
                    ESP_LOGW(TAG, "ADS start error: phase=%d, i2c_err=%lu, ads_err=%lu",
                             phase, (unsigned long)daq->i2c_err_count,
                             (unsigned long)daq->ads_err_count);
                    daq->last_err_log_time = now;
                }
                continue;
            }

            // Step 2: Wait for conversion complete — NO mutex held
            if (daq->alert_gpio >= 0) {
                // Interrupt mode: ALERT/RDY ISR signals the semaphore (~1.16ms at 860 SPS)
                if (xSemaphoreTake(daq->conv_ready_sem, pdMS_TO_TICKS(50)) != pdTRUE) {
                    daq->ads_err_count++;
                    int64_t now = esp_timer_get_time();
                    if (now - daq->last_err_log_time > 5000000) {
                        ESP_LOGW(TAG, "ALERT/RDY timeout: phase=%d isr_total=%lu ads_err=%lu",
                                 phase, (unsigned long)daq->isr_fire_count,
                                 (unsigned long)daq->ads_err_count);
                        daq->last_err_log_time = now;
                    }
                    continue;
                }
                ESP_LOGV(TAG, "  [ph%d] ISR fired — isr_total=%lu",
                         phase, (unsigned long)daq->isr_fire_count);
            } else {
                // Polling mode: fixed wait > 1.16ms conversion time at 860 SPS
                vTaskDelay(pdMS_TO_TICKS(2));
            }

            // Step 3: Read result (~100 µs mutex hold)
            ret = i2c_mux_mgr_exec(daq->mux_mgr, daq->tca_channel,
                                    ads_read_result_callback, &ctx);
            if (ret != ESP_OK || ctx.result != ESP_OK) {
                int64_t now = esp_timer_get_time();
                if (now - daq->last_err_log_time > 5000000) {
                    ESP_LOGW(TAG, "ADS read error: phase=%d, i2c_err=%lu, ads_err=%lu",
                             phase, (unsigned long)daq->i2c_err_count,
                             (unsigned long)daq->ads_err_count);
                    daq->last_err_log_time = now;
                }
                continue;
            }

            // Convert to voltage
            float voltage = raw_to_voltage(ctx.raw_value);
            ESP_LOGV(TAG, "  [ph%d] raw=%d  voltage=%.4fV", phase, ctx.raw_value, voltage);

            // Clamp negative values (shouldn't happen with unipolar, but safety)
            if (voltage < 0) voltage = 0;

            // Accumulate
            daq->v_sum_sq[phase] += voltage * voltage;
            daq->v_sum[phase] += voltage;
            if (voltage > daq->v_max[phase]) {
                daq->v_max[phase] = voltage;
            }
            daq->phase_sample_count[phase]++;
            phase_ok = true;
        }

        // Only advance the window counter when at least one phase succeeded.
        // Prevents process_window being called with all-zero data during sustained
        // I2C errors, which would produce Vrms=0 and trigger false CT_OPEN.
        if (phase_ok) {
            daq->sample_count++;
        }

        // Process window when full
        if (daq->sample_count >= CT_WINDOW_SAMPLES) {
            process_window(daq);
        }

        // Yield to lower-priority tasks (HMI, priority 4) after every 3-phase cycle.
        //
        // Problem: ADS1115 at 860 SPS fires the DRDY ISR every ~1.16 ms.
        // DAQ's start-conversion callback takes ~400 µs (holding the mutex).
        // After the mutex is released and before xSemaphoreTake(conv_ready_sem)
        // is called, the ISR can already have pre-loaded the semaphore with a token.
        // In that case xSemaphoreTake returns instantly — no blocking, no scheduler
        // context switch — and DAQ (priority 5) re-acquires the mutex immediately
        // for the next phase.  HMI (priority 4) never gets the CPU to take the mutex,
        // producing 200 ms mutex timeouts for LCD and keypad even though i2c_err=0.
        //
        // Fix: a mandatory 1 ms delay once per 3-phase cycle guarantees the scheduler
        // runs HMI.  Cost: cycle time 4 ms → 5 ms, effective rate ~238 → ~200 SPS per
        // channel.  Still 4× above the 50 Hz Nyquist frequency — CT accuracy unaffected.
        if (!phase_ok) {
            vTaskDelay(pdMS_TO_TICKS(10));  // all phases failed: longer yield
        } else {
            vTaskDelay(pdMS_TO_TICKS(1));   // normal: 1 ms yield lets HMI take mutex
        }
    }

    esp_task_wdt_delete(NULL);
    ESP_LOGI(TAG, "DAQ task stopped");
    xSemaphoreGive(daq->task_done_sem);  // Signal deinit that we have fully exited
    vTaskDelete(NULL);
}

void daq_ct_get_latest(daq_ct_handle_t *handle, daq_snapshot_t *out)
{
    if (handle == NULL || out == NULL) return;

    if (xSemaphoreTake(handle->snapshot_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        memcpy(out, &handle->snapshot, sizeof(daq_snapshot_t));
        xSemaphoreGive(handle->snapshot_mutex);
    } else {
        ESP_LOGW(TAG, "daq_ct_get_latest: mutex timeout — returning stale snapshot");
        // out retains caller's previous value; stale data is safe (seq unchanged)
    }
}

daq_ct_handle_t *daq_ct_get_handle(void)
{
    return g_daq_handle;
}

esp_err_t daq_ct_update_cal_factors(daq_ct_handle_t *handle, const float cal[3])
{
    if (handle == NULL || cal == NULL) return ESP_ERR_INVALID_ARG;

    // snapshot_mutex is held for ~1 µs — safe to call from any task or MQTT callback
    if (xSemaphoreTake(handle->snapshot_mutex, pdMS_TO_TICKS(10)) != pdTRUE) {
        ESP_LOGE(TAG, "daq_ct_update_cal_factors: mutex timeout — cal_factor NOT updated");
        return ESP_ERR_TIMEOUT;
    }
    for (int i = 0; i < 3; i++) {
        if (cal[i] > 0) {
            handle->cal_factor[i] = cal[i];
        }
    }
    float a = handle->cal_factor[0], b = handle->cal_factor[1], c = handle->cal_factor[2];
    xSemaphoreGive(handle->snapshot_mutex);

    ESP_LOGI(TAG, "cal_factor updated live: A=%.4f B=%.4f C=%.4f", a, b, c);
    return ESP_OK;
}

esp_err_t daq_ct_deinit(daq_ct_handle_t *handle)
{
    if (handle == NULL) return ESP_ERR_INVALID_ARG;

    handle->running = false;
    // Wake the DAQ task immediately if it is blocked on conv_ready_sem
    // (without this, deinit would wait up to 50ms timeout per phase = 150ms per sample + margin)
    xSemaphoreGive(handle->conv_ready_sem);

    // Wait for the DAQ task to fully exit before freeing any resources.
    // task_done_sem is Given by daq_ct_task() just before vTaskDelete(NULL).
    // The previous vTaskDelay(100ms) was a race: under I2C error conditions one
    // 3-phase cycle can take up to 150ms (3 × 50ms ALERT timeout), so the delay
    // expired while the task was still running → use-after-free on conv_ready_sem
    // and handle->mux_mgr on the very next provisioning restart.
    xSemaphoreTake(handle->task_done_sem, portMAX_DELAY);

    // Only remove the ISR handler if interrupt mode was configured (alert_gpio >= 0).
    // Passing an invalid GPIO number to gpio_isr_handler_remove() causes an assert.
    if (handle->alert_gpio >= 0) {
        gpio_isr_handler_remove(handle->alert_gpio);
    }

    if (handle->ads_handle != NULL) {
        i2c_master_bus_rm_device(handle->ads_handle);
        handle->ads_handle = NULL;
    }

    if (handle->conv_ready_sem != NULL) {
        vSemaphoreDelete(handle->conv_ready_sem);
        handle->conv_ready_sem = NULL;
    }

    if (handle->snapshot_mutex != NULL) {
        vSemaphoreDelete(handle->snapshot_mutex);
        handle->snapshot_mutex = NULL;
    }

    if (handle->task_done_sem != NULL) {
        vSemaphoreDelete(handle->task_done_sem);
        handle->task_done_sem = NULL;
    }

    if (g_daq_handle == handle) {
        g_daq_handle = NULL;
    }

    free(handle);
    return ESP_OK;
}
