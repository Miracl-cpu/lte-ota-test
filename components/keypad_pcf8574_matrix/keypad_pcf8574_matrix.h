/**
 * @file keypad_pcf8574_matrix.h
 * @brief 4x4 Matrix Keypad driver via PCF8574 I2C expander
 *
 * IMPORTANT (matches OLD working driver / actual scan):
 *   - Rows are driven on LOW nibble:   P0..P3 = R1..R4 (active-low)
 *   - Columns are read on HIGH nibble: P4..P7 = C1..C4 (active-low)
 *
 * Keypad layout:
 *       C1  C2  C3  C4
 *   R1:  1   2   3   A
 *   R2:  4   5   6   B
 *   R3:  7   8   9   C
 *   R4:  *   0   #   D
 */

#ifndef KEYPAD_PCF8574_MATRIX_H
#define KEYPAD_PCF8574_MATRIX_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "i2c_mux_mgr.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    KEY_DOWN = 0,
    KEY_UP   = 1
} key_event_type_t;

typedef struct {
    char key;
    key_event_type_t type;
    uint32_t ts_ms;
} key_event_t;

typedef struct {
    i2c_mux_mgr_t *mux_mgr;
    i2c_master_dev_handle_t pcf_handle;
    uint8_t pcf_addr;
    uint8_t tca_channel;

    /* debouncing state (per instance) */
    uint16_t prev_raw;
    uint8_t  stable_count;

    /* latest scan result */
    uint16_t raw_state;      // 16-bit bitmap: bit (row*4+col)

    /* event generation */
    char last_key;
    bool key_pressed;

    /* optional pending event (not strictly needed, kept for compatibility) */
    key_event_t pending_event;
    bool has_pending_event;

} keypad_t;

esp_err_t keypad_init(keypad_t *keypad, i2c_mux_mgr_t *mux_mgr,
                      uint8_t pcf_addr, uint8_t tca_channel);

/**
 * @brief Deinitialize the keypad and release the I2C device handle
 *
 * Must be called before re-initializing or when the keypad is no longer needed.
 * Removes the PCF8574 device from the I2C master bus so the handle can be
 * safely reused (e.g. after an OTA reboot or error recovery).
 *
 * @param keypad Pointer to keypad handle
 * @return ESP_OK on success
 */
esp_err_t keypad_deinit(keypad_t *keypad);

esp_err_t keypad_scan(keypad_t *keypad);

/**
 * @brief Poll for key events (KEY_DOWN / KEY_UP)
 *
 * Call every ~20–30ms.
 * Returns true when an event is produced.
 */
bool keypad_poll(keypad_t *keypad, key_event_t *ev);

#ifdef __cplusplus
}
#endif

#endif // KEYPAD_PCF8574_MATRIX_H
