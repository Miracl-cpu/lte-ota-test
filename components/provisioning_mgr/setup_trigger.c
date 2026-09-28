/**
 * @file setup_trigger.c
 * @brief Setup trigger state machine implementation
 *
 * STATE MACHINE LOGIC:
 *
 * NORMAL → HASH_HOLDING:
 *   - When '#' key is pressed down
 *   - Record start time
 *
 * HASH_HOLDING → NORMAL:
 *   - When '#' key is released before 5 seconds
 *   - Reset state
 *
 * HASH_HOLDING → PASSWORD_ENTRY:
 *   - When '#' has been held for >= 5 seconds
 *   - Start password timeout (30s)
 *   - Clear password buffer
 *
 * PASSWORD_ENTRY → NORMAL (via ACCESS_DENIED):
 *   - When wrong password entered (press 'A' to submit)
 *   - Show "Access Denied" for 2 seconds
 *
 * PASSWORD_ENTRY → NORMAL (via TIMEOUT):
 *   - When 30 seconds elapsed without valid password
 *   - Show "Timeout" for 2 seconds
 *
 * PASSWORD_ENTRY → NORMAL:
 *   - When '*' pressed (cancel)
 *   - Immediate return to normal
 *
 * PASSWORD_ENTRY → ENTERING_SETUP:
 *   - When correct password "7552" entered and 'A' pressed
 *
 * ACCESS_DENIED/TIMEOUT → NORMAL:
 *   - After 2 second message display
 *
 * KEY HANDLING IN PASSWORD_ENTRY:
 *   '0'-'9': Add digit to password (if pos < 4)
 *   'A': Submit password for validation
 *   'D': Backspace (delete last digit)
 *   '*': Cancel, return to normal
 *   '#', 'B', 'C': Ignored
 */

#include "setup_trigger.h"
#include "log_config.h"
#undef LOG_LOCAL_LEVEL
#define LOG_LOCAL_LEVEL LOG_LEVEL_SETUP_TRIGGER
#include "esp_log.h"
#include <string.h>

static const char *TAG = "setup_trigger";

// Internal state
static setup_trigger_ctx_t s_ctx;

void setup_trigger_init(void)
{
    memset(&s_ctx, 0, sizeof(s_ctx));
    s_ctx.state = PROV_STATE_NORMAL;
    s_ctx.hash_key_down = false;
    s_ctx.ignore_hash_until_release = false;
    ESP_LOGI(TAG, "Setup trigger initialized");
}

void setup_trigger_reset(void)
{
    s_ctx.state = PROV_STATE_NORMAL;
    s_ctx.hash_key_down = false;
    s_ctx.hash_hold_start_ms = 0;
    s_ctx.message_start_ms = 0;
    s_ctx.ignore_hash_until_release = false;
    memset(&s_ctx.password, 0, sizeof(s_ctx.password));
    ESP_LOGI(TAG, "Setup trigger reset");
}

static void clear_password(void)
{
    memset(&s_ctx.password, 0, sizeof(s_ctx.password));
    s_ctx.password.pos = 0;
}

static bool validate_password(void)
{
    // Check if entered password matches SETUP_PASSWORD
    return (s_ctx.password.pos == SETUP_PASSWORD_LEN) &&
           (strcmp(s_ctx.password.digits, SETUP_PASSWORD) == 0);
}

bool setup_trigger_process_key(char key, bool pressed, uint32_t now_ms)
{
    // State-specific key handling
    switch (s_ctx.state) {
        case PROV_STATE_NORMAL:
            // Only care about '#' key press
            if (key == '#') {
                if (pressed) {
                    ESP_LOGI(TAG, "'#' pressed, starting hold detection");
                    s_ctx.hash_key_down = true;
                    s_ctx.hash_hold_start_ms = now_ms;
                    s_ctx.state = PROV_STATE_HASH_HOLDING;
                    return true;  // Consume the key
                }
            }
            return false;  // Don't consume

        case PROV_STATE_HASH_HOLDING:
            if (key == '#') {
                if (!pressed) {
                    // '#' released before timeout - cancel
                    ESP_LOGI(TAG, "'#' released before 5s, canceling");
                    s_ctx.hash_key_down = false;
                    s_ctx.state = PROV_STATE_NORMAL;
                }
                return true;  // Consume '#' keys during hold
            }
            // Other keys during hold - cancel the trigger
            if (pressed) {
                ESP_LOGI(TAG, "Other key pressed during hold, canceling");
                s_ctx.state = PROV_STATE_NORMAL;
                s_ctx.hash_key_down = false;
                return true;  // Consume the key - don't let it leak to normal HMI
            }
            return true;  // Consume key releases too during hold detection

        case PROV_STATE_PASSWORD_ENTRY:
            // Handle '#' key - just clear the ignore flag on release (no escape function)
            if (key == '#') {
                if (!pressed) {
                    // '#' released - clear the ignore flag
                    if (s_ctx.ignore_hash_until_release) {
                        ESP_LOGI(TAG, "'#' released after entry hold");
                        s_ctx.ignore_hash_until_release = false;
                    }
                }
                return true;  // Consume '#' but don't use as escape
            }

            // '*' is the escape/cancel key
            if (key == '*') {
                if (pressed) {
                    ESP_LOGI(TAG, "Password entry canceled via '*'");
                    s_ctx.state = PROV_STATE_NORMAL;
                    clear_password();
                }
                return true;
            }

            if (!pressed) {
                return false;  // Ignore other key releases in password entry
            }

            if (key >= '0' && key <= '9') {
                // Add digit if room
                if (s_ctx.password.pos < SETUP_PASSWORD_LEN) {
                    s_ctx.password.digits[s_ctx.password.pos++] = key;
                    s_ctx.password.digits[s_ctx.password.pos] = '\0';
                    ESP_LOGI(TAG, "Password digit added, pos=%d", s_ctx.password.pos);
                }
                return true;
            }

            if (key == 'A') {
                // Submit password
                ESP_LOGI(TAG, "Password submitted: %s", s_ctx.password.digits);
                if (validate_password()) {
                    ESP_LOGI(TAG, "Password correct! Entering setup mode");
                    s_ctx.state = PROV_STATE_ENTERING_SETUP;
                } else {
                    ESP_LOGW(TAG, "Password incorrect!");
                    s_ctx.state = PROV_STATE_ACCESS_DENIED;
                    s_ctx.message_start_ms = now_ms;
                }
                return true;
            }

            if (key == 'D') {
                // Backspace
                if (s_ctx.password.pos > 0) {
                    s_ctx.password.pos--;
                    s_ctx.password.digits[s_ctx.password.pos] = '\0';
                    ESP_LOGI(TAG, "Password backspace, pos=%d", s_ctx.password.pos);
                }
                return true;
            }

            // 'B', 'C' ignored
            return true;  // Consume but ignore

        case PROV_STATE_SETUP_ACTIVE:
            // In setup mode, Enter exits and reboots.
            if (key == '#' && pressed) {
                ESP_LOGI(TAG, "Enter pressed in setup mode, exiting");
                s_ctx.state = PROV_STATE_EXITING_SETUP;
            } else if (key == '*' && pressed) {
                ESP_LOGI(TAG, "Back pressed in setup mode, returning to menu");
                s_ctx.state = PROV_STATE_NORMAL;
            }
            return true;  // Consume all keys in setup mode

        case PROV_STATE_ACCESS_DENIED:
        case PROV_STATE_TIMEOUT:
        case PROV_STATE_ENTERING_SETUP:
        case PROV_STATE_EXITING_SETUP:
        case PROV_STATE_REBOOTING:
            // Consume all keys during these transitional states
            return true;

        default:
            return false;
    }
}

void setup_trigger_update(uint32_t now_ms)
{
    s_ctx.last_update_ms = now_ms;

    switch (s_ctx.state) {
        case PROV_STATE_HASH_HOLDING:
            // Check if held long enough
            if (s_ctx.hash_key_down) {
                uint32_t held_ms = now_ms - s_ctx.hash_hold_start_ms;
                if (held_ms >= HASH_HOLD_TIME_MS) {
                    ESP_LOGI(TAG, "'#' held for 5s, entering password entry");
                    s_ctx.state = PROV_STATE_PASSWORD_ENTRY;
                    // Clear password first, then set timestamp (clear_password memsets the struct)
                    clear_password();
                    s_ctx.password.entry_start_ms = now_ms;
                    // Prevent the held '#' from immediately acting as Escape
                    s_ctx.ignore_hash_until_release = true;
                }
            }
            break;

        case PROV_STATE_PASSWORD_ENTRY:
            // Check for timeout
            if ((now_ms - s_ctx.password.entry_start_ms) >= PASSWORD_TIMEOUT_MS) {
                ESP_LOGW(TAG, "Password entry timeout");
                s_ctx.state = PROV_STATE_TIMEOUT;
                s_ctx.message_start_ms = now_ms;
                clear_password();
            }
            break;

        case PROV_STATE_ACCESS_DENIED:
        case PROV_STATE_TIMEOUT:
            // Check if message display time elapsed
            if ((now_ms - s_ctx.message_start_ms) >= MESSAGE_DISPLAY_MS) {
                ESP_LOGI(TAG, "Returning to normal mode");
                s_ctx.state = PROV_STATE_NORMAL;
                clear_password();
            }
            break;

        case PROV_STATE_SETUP_ACTIVE:
            // Check inactivity timeout (5 minutes)
            if (s_ctx.setup_last_activity_ms > 0) {
                uint32_t inactive_ms = now_ms - s_ctx.setup_last_activity_ms;
                if (inactive_ms >= SETUP_INACTIVITY_TIMEOUT_MS) {
                    ESP_LOGW(TAG, "Setup mode inactivity timeout (%lu ms), auto-exiting",
                             (unsigned long)inactive_ms);
                    s_ctx.state = PROV_STATE_EXITING_SETUP;
                }
            }
            break;

        default:
            break;
    }
}

prov_state_t setup_trigger_get_state(void)
{
    return s_ctx.state;
}

void setup_trigger_set_state(prov_state_t state, uint32_t now_ms)
{
    s_ctx.state = state;
    if (state == PROV_STATE_EXITING_SETUP || state == PROV_STATE_REBOOTING) {
        s_ctx.message_start_ms = now_ms;
    }
    if (state == PROV_STATE_SETUP_ACTIVE) {
        // Reset hash key tracking to prevent immediate exit from old hold
        s_ctx.hash_key_down = false;
        s_ctx.hash_hold_start_ms = 0;
        // Start inactivity timer
        s_ctx.setup_last_activity_ms = now_ms;
    }
}

void setup_trigger_get_password_mask(char *mask_out, size_t len)
{
    if (mask_out == NULL || len < 5) {
        return;
    }

    // Build mask like "**__" where * = entered, _ = remaining
    for (int i = 0; i < SETUP_PASSWORD_LEN && i < (int)(len - 1); i++) {
        if (i < s_ctx.password.pos) {
            mask_out[i] = '*';
        } else {
            mask_out[i] = '_';
        }
    }
    mask_out[SETUP_PASSWORD_LEN] = '\0';
}

uint32_t setup_trigger_get_hold_progress(void)
{
    if (s_ctx.state != PROV_STATE_HASH_HOLDING) {
        return 0;
    }

    if (!s_ctx.hash_key_down) {
        return 0;
    }

    uint32_t held = s_ctx.last_update_ms - s_ctx.hash_hold_start_ms;
    return held;
}

uint32_t setup_trigger_get_timeout_remaining(uint32_t now_ms)
{
    if (s_ctx.state != PROV_STATE_PASSWORD_ENTRY) {
        return 0;
    }

    uint32_t elapsed = now_ms - s_ctx.password.entry_start_ms;
    if (elapsed >= PASSWORD_TIMEOUT_MS) {
        return 0;
    }
    return PASSWORD_TIMEOUT_MS - elapsed;
}

void setup_trigger_reset_activity(uint32_t now_ms)
{
    if (s_ctx.state == PROV_STATE_SETUP_ACTIVE) {
        s_ctx.setup_last_activity_ms = now_ms;
    }
}

bool setup_trigger_is_detecting(void)
{
    return s_ctx.state == PROV_STATE_HASH_HOLDING;
}
