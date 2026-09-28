/**
 * @file keypad_pcf8574_matrix.c
 * @brief 4x4 Matrix Keypad driver via PCF8574 I2C expander
 *
 * Scan method (matches OLD working driver):
 * - Rows on P0..P3 (LOW nibble): drive one row LOW at a time (active-low)
 * - Columns on P4..P7 (HIGH nibble): read which column goes LOW (active-low)
 *
 * PCF8574 quasi-bidirectional rule:
 * - Write '1' to a pin to release it (input/high)
 * - Write '0' to drive it low
 */

#include "keypad_pcf8574_matrix.h"
#include "log_config.h"
#undef LOG_LOCAL_LEVEL
#define LOG_LOCAL_LEVEL LOG_LEVEL_KEYPAD
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include <string.h>

static const char *TAG = "keypad";

/* Low nibble rows, high nibble columns */
#define ROW_MASK   0x0F  // P0..P3
#define COL_MASK   0xF0  // P4..P7

/* Debounce: require N stable scans after a raw change. The HMI polls every
 * 20 ms, so 1 stable scan gives responsive ~40 ms key-down detection. */
#define DEBOUNCE_SCANS  1

/* Key map [row][col] */
static const char KEY_MAP[4][4] = {
    {'1', '2', '3', 'A'},
    {'4', '5', '6', 'B'},
    {'7', '8', '9', 'C'},
    {'*', '0', '#', 'D'}
};

typedef struct {
    keypad_t *keypad;
    uint16_t key_state;
    esp_err_t result;
} keypad_scan_ctx_t;

static char find_pressed_key(uint16_t state)
{
    for (int row = 0; row < 4; row++) {
        for (int col = 0; col < 4; col++) {
            if (state & (1u << (row * 4 + col))) {
                return KEY_MAP[row][col];
            }
        }
    }
    return '\0';
}

static esp_err_t keypad_scan_callback(void *ctx)
{
    keypad_scan_ctx_t *sctx = (keypad_scan_ctx_t *)ctx;
    keypad_t *keypad = sctx->keypad;
    uint16_t state = 0;

    /*
     * For each row:
     *   row_pattern = ~(1<<row) & 0x0F   => one row low, others high
     *   write (row_pattern | 0xF0)      => release all columns high
     *   read back; columns are in upper nibble and active-low
     */
    ESP_LOGV(TAG, "--- Scan start ---");
    for (int row = 0; row < 4; row++) {
        uint8_t row_pattern = (uint8_t)(~(1u << row)) & 0x0F;
        uint8_t out = (uint8_t)(row_pattern | 0xF0);

        esp_err_t ret = i2c_master_transmit(keypad->pcf_handle, &out, 1, pdMS_TO_TICKS(20));
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Row%d TX failed (wrote=0x%02X): %s", row, out, esp_err_to_name(ret));
            sctx->result = ret;
            return ret;
        }

        uint8_t in = 0xFF;
        ret = i2c_master_receive(keypad->pcf_handle, &in, 1, pdMS_TO_TICKS(20));
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Row%d RX failed: %s", row, esp_err_to_name(ret));
            sctx->result = ret;
            return ret;
        }

        uint8_t col_data = (in >> 4) & 0x0F;  // C1..C4 = bits 0..3 here
        ESP_LOGV(TAG, "Row%d: TX=0x%02X RX=0x%02X col_nibble=0x%X", row, out, in, col_data);

        for (int col = 0; col < 4; col++) {
            if (!(col_data & (1u << col))) {
                state |= (1u << (row * 4 + col));
            }
        }
    }

    /* release all pins high */
    uint8_t release = 0xFF;
    esp_err_t rel_ret = i2c_master_transmit(keypad->pcf_handle, &release, 1, pdMS_TO_TICKS(20));
    ESP_LOGV(TAG, "Release: TX=0x%02X ret=%s", release, esp_err_to_name(rel_ret));
    (void)rel_ret;

    sctx->key_state = state;
    sctx->result = ESP_OK;
    return ESP_OK;
}

esp_err_t keypad_init(keypad_t *keypad, i2c_mux_mgr_t *mux_mgr,
                      uint8_t pcf_addr, uint8_t tca_channel)
{
    if (keypad == NULL || mux_mgr == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(keypad, 0, sizeof(*keypad));
    keypad->mux_mgr = mux_mgr;
    keypad->pcf_addr = pcf_addr;
    keypad->tca_channel = tca_channel;

    keypad->prev_raw = 0;
    keypad->stable_count = 0;
    keypad->raw_state = 0;

    keypad->last_key = '\0';
    keypad->key_pressed = false;
    keypad->has_pending_event = false;

    esp_err_t ret = i2c_mux_mgr_add_device(mux_mgr, pcf_addr, &keypad->pcf_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add keypad PCF8574 @0x%02X: %s", pcf_addr, esp_err_to_name(ret));
        return ret;
    }

    /* Initialize port: all released high — failure means PCF8574 is absent or stuck */
    keypad_scan_ctx_t ctx = { .keypad = keypad, .key_state = 0, .result = ESP_OK };
    ret = i2c_mux_mgr_exec(mux_mgr, tca_channel, keypad_scan_callback, &ctx);
    if (ret != ESP_OK || ctx.result != ESP_OK) {
        esp_err_t err = (ret != ESP_OK) ? ret : ctx.result;
        ESP_LOGE(TAG, "Keypad PCF8574 @0x%02X init scan failed: %s", pcf_addr, esp_err_to_name(err));
        i2c_master_bus_rm_device(keypad->pcf_handle);
        keypad->pcf_handle = NULL;
        return err;
    }

    ESP_LOGI(TAG, "Keypad initialized: PCF@0x%02X on TCA CH%d", pcf_addr, tca_channel);
    return ESP_OK;
}

esp_err_t keypad_deinit(keypad_t *keypad)
{
    if (keypad == NULL) return ESP_ERR_INVALID_ARG;

    if (keypad->pcf_handle != NULL) {
        i2c_master_bus_rm_device(keypad->pcf_handle);
        keypad->pcf_handle = NULL;
    }

    return ESP_OK;
}

esp_err_t keypad_scan(keypad_t *keypad)
{
    if (keypad == NULL) return ESP_ERR_INVALID_ARG;

    keypad_scan_ctx_t ctx = { .keypad = keypad, .key_state = 0, .result = ESP_OK };
    esp_err_t ret = i2c_mux_mgr_exec(keypad->mux_mgr, keypad->tca_channel,
                                     keypad_scan_callback, &ctx);
    if (ret != ESP_OK) return ret;
    if (ctx.result != ESP_OK) return ctx.result;

    keypad->raw_state = ctx.key_state;
    return ESP_OK;
}

bool keypad_poll(keypad_t *keypad, key_event_t *ev)
{
    if (keypad == NULL || ev == NULL) return false;

    /* return pending event if used */
    if (keypad->has_pending_event) {
        *ev = keypad->pending_event;
        keypad->has_pending_event = false;
        return true;
    }

    esp_err_t scan_ret = keypad_scan(keypad);
    if (scan_ret != ESP_OK) {
        ESP_LOGW(TAG, "Scan I2C error: %s", esp_err_to_name(scan_ret));
        return false;
    }

    uint16_t raw = keypad->raw_state;

    /* debounce */
    if (raw == keypad->prev_raw) {
        if (keypad->stable_count < 255) keypad->stable_count++;
    } else {
        keypad->stable_count = 0;
        keypad->prev_raw = raw;
        return false;
    }

    if (keypad->stable_count < DEBOUNCE_SCANS) {
        return false;
    }

    char current_key = find_pressed_key(raw);
    uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);

    if (current_key != '\0' && !keypad->key_pressed) {
        keypad->key_pressed = true;
        keypad->last_key = current_key;

        ev->key = current_key;
        ev->type = KEY_DOWN;
        ev->ts_ms = now_ms;

        ESP_LOGD(TAG, "Key DOWN: '%c'", current_key);
        return true;
    }

    if (current_key == '\0' && keypad->key_pressed) {
        keypad->key_pressed = false;

        ev->key = keypad->last_key;
        ev->type = KEY_UP;
        ev->ts_ms = now_ms;

        ESP_LOGD(TAG, "Key UP: '%c'", keypad->last_key);
        keypad->last_key = '\0';
        return true;
    }

    return false;
}
