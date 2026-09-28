/**
 * @file setup_trigger.h
 * @brief Setup trigger state machine (internal)
 *
 * Handles:
 * - '#' key hold detection (5 seconds)
 * - Password entry state machine
 * - Timeout management
 */

#ifndef SETUP_TRIGGER_H
#define SETUP_TRIGGER_H

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include "provisioning_mgr.h"

#ifdef __cplusplus
extern "C" {
#endif

// Password entry tracking
typedef struct {
    char digits[SETUP_PASSWORD_LEN + 1];  // Entered digits + null
    uint8_t pos;                           // Current position (0-4)
    uint32_t entry_start_ms;               // Timestamp when entry started
} password_entry_t;

// Trigger context (internal)
typedef struct {
    prov_state_t state;
    uint32_t hash_hold_start_ms;           // When '#' press started
    uint32_t message_start_ms;             // For timed messages
    password_entry_t password;
    bool hash_key_down;                    // Current '#' key state
    uint32_t last_update_ms;               // Last update timestamp
    bool ignore_hash_until_release;        // Prevents entry-hold from acting as escape/exit
    uint32_t setup_last_activity_ms;       // Last web/client activity in setup mode
} setup_trigger_ctx_t;

/**
 * @brief Initialize setup trigger
 */
void setup_trigger_init(void);

/**
 * @brief Process key for trigger detection
 * @param key Key character
 * @param pressed true=down, false=up
 * @param now_ms Current time
 * @return true if key was consumed
 */
bool setup_trigger_process_key(char key, bool pressed, uint32_t now_ms);

/**
 * @brief Update trigger state (call periodically)
 * @param now_ms Current time
 */
void setup_trigger_update(uint32_t now_ms);

/**
 * @brief Get current trigger state
 */
prov_state_t setup_trigger_get_state(void);

/**
 * @brief Set trigger state (used by provisioning_mgr)
 */
void setup_trigger_set_state(prov_state_t state, uint32_t now_ms);

/**
 * @brief Get password entry mask for display
 * @param mask_out Buffer for mask string (e.g., "**__")
 * @param len Buffer length (min 8)
 */
void setup_trigger_get_password_mask(char *mask_out, size_t len);

/**
 * @brief Get hold progress in milliseconds
 */
uint32_t setup_trigger_get_hold_progress(void);

/**
 * @brief Get password timeout remaining in milliseconds
 */
uint32_t setup_trigger_get_timeout_remaining(uint32_t now_ms);

/**
 * @brief Reset trigger to idle state
 */
void setup_trigger_reset(void);

/**
 * @brief Reset setup inactivity timer (call on web request or client connect)
 * @param now_ms Current time
 */
void setup_trigger_reset_activity(uint32_t now_ms);

/**
 * @brief Check if setup trigger detection is active
 * @return true if actively detecting '#' hold
 */
bool setup_trigger_is_detecting(void);

#ifdef __cplusplus
}
#endif

#endif // SETUP_TRIGGER_H
