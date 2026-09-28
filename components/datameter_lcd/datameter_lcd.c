#include "datameter_lcd.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_timer.h"
#include <limits.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

// ST7920 serial pins. CS/RST follow the board GPIO map; data/clock remain on
// the existing wired pins.
#define PIN_LCD_RS_CS GPIO_NUM_1
#define PIN_LCD_RW_DATA GPIO_NUM_11
#define PIN_LCD_E_CLK GPIO_NUM_12
#define PIN_LCD_RESET GPIO_NUM_2

// --- GPIO & Callbacks ---
static void lcd_gpio_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << PIN_LCD_RS_CS) | (1ULL << PIN_LCD_RW_DATA) |
                        (1ULL << PIN_LCD_E_CLK) | (1ULL << PIN_LCD_RESET),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&io_conf);
    gpio_set_level(PIN_LCD_RESET, 1);
}

static uint8_t u8g2_esp32_gpio_and_delay_cb(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
    switch (msg)
    {
    case U8X8_MSG_GPIO_AND_DELAY_INIT:
        lcd_gpio_init();
        break;
    case U8X8_MSG_DELAY_MILLI:
        vTaskDelay(pdMS_TO_TICKS(arg_int));
        break;
    case U8X8_MSG_DELAY_10MICRO:
        esp_rom_delay_us(arg_int * 10);
        break;
    case U8X8_MSG_GPIO_SPI_CLOCK:
        gpio_set_level(PIN_LCD_E_CLK, arg_int);
        break;
    case U8X8_MSG_GPIO_SPI_DATA:
        gpio_set_level(PIN_LCD_RW_DATA, arg_int);
        break;
    case U8X8_MSG_GPIO_CS:
        gpio_set_level(PIN_LCD_RS_CS, arg_int);
        break;
    case U8X8_MSG_GPIO_RESET:
        gpio_set_level(PIN_LCD_RESET, arg_int);
        break;
    }
    return 1;
}

/* One-slot mailbox: UI replaces obsolete frames rather than waiting for SPI.
 * All producers hold the recursive drawing mutex; the worker owns GPIO writes. */
typedef struct {
    u8g2_t context;
    uint8_t pixels[1024];
    bool power_save;
} lcd_frame_t;
static QueueHandle_t s_lcd_frames;
static SemaphoreHandle_t s_lcd_draw_mutex;
static lcd_frame_t s_lcd_staging;
static portMUX_TYPE s_lcd_stats_mux = portMUX_INITIALIZER_UNLOCKED;
static uint32_t s_lcd_completed, s_lcd_slow, s_lcd_recoveries;
static uint32_t s_lcd_last_transfer_ms;

static void lcd_submit_frame(u8g2_t *d) {
    if (!s_lcd_frames || !d) return;
    size_t size = (size_t)u8g2_GetBufferTileWidth(d) * u8g2_GetBufferTileHeight(d) * 8;
    if (size != sizeof(s_lcd_staging.pixels)) return;
    s_lcd_staging.context = *d;
    memcpy(s_lcd_staging.pixels, u8g2_GetBufferPtr(d), size);
    s_lcd_staging.power_save = false;
    xQueueOverwrite(s_lcd_frames, &s_lcd_staging);
}

static void lcd_transfer_worker(void *arg) {
    lcd_frame_t frame;
    uint32_t slow_streak = 0;
    int64_t last_recovery_us = 0;
    while (1) {
        if (xQueueReceive(s_lcd_frames, &frame, portMAX_DELAY) != pdTRUE) continue;
        frame.context.tile_buf_ptr = frame.pixels;
        if (frame.power_save) {
            u8g2_SetPowerSave(&frame.context, 1);
            continue;
        }
        int64_t start = esp_timer_get_time();
        /* Recovery is serialized here, never performed from another task.
         * This detects slow transfers, not corrupt pixels on a write-only LCD. */
        if (slow_streak >= 3 && start - last_recovery_us >= 60000000) {
            u8g2_InitDisplay(&frame.context);
            u8g2_SetPowerSave(&frame.context, 0);
            last_recovery_us = start;
            slow_streak = 0;
            portENTER_CRITICAL(&s_lcd_stats_mux);
            s_lcd_recoveries++;
            portEXIT_CRITICAL(&s_lcd_stats_mux);
        }
        u8g2_SendBuffer(&frame.context);
        uint32_t elapsed = (uint32_t)((esp_timer_get_time() - start) / 1000);
        slow_streak = elapsed >= 500 ? slow_streak + 1 : 0;
        portENTER_CRITICAL(&s_lcd_stats_mux);
        s_lcd_last_transfer_ms = elapsed;
        s_lcd_completed++;
        if (elapsed >= 500) s_lcd_slow++;
        portEXIT_CRITICAL(&s_lcd_stats_mux);
    }
}

void lcd_get_runtime_stats(uint32_t *completed, uint32_t *slow,
                           uint32_t *recoveries, uint32_t *last_ms) {
    portENTER_CRITICAL(&s_lcd_stats_mux);
    *completed = s_lcd_completed;
    *slow = s_lcd_slow;
    *recoveries = s_lcd_recoveries;
    *last_ms = s_lcd_last_transfer_ms;
    portEXIT_CRITICAL(&s_lcd_stats_mux);
}

// --- Icons ---

static void draw_signal_icon(u8g2_t *d, int x, int y, uint8_t level)
{
    for (int i = 0; i < 4; i++)
    {
        int h = 3 + (i * 2);
        if (i < level)
            u8g2_DrawBox(d, x + (i * 5), y + (9 - h), 3, h);
        else
            u8g2_DrawFrame(d, x + (i * 5), y + (9 - h), 3, h);
    }
}

static void draw_wifi_icon(u8g2_t *d, int x, int y, uint8_t level)
{
    (void)level;

    u8g2_DrawLine(d, x + 1, y + 4, x + 3, y + 2);
    u8g2_DrawLine(d, x + 3, y + 2, x + 6, y + 1);
    u8g2_DrawLine(d, x + 6, y + 1, x + 8, y + 1);
    u8g2_DrawLine(d, x + 8, y + 1, x + 11, y + 2);
    u8g2_DrawLine(d, x + 11, y + 2, x + 13, y + 4);

    u8g2_DrawLine(d, x + 4, y + 7, x + 6, y + 5);
    u8g2_DrawLine(d, x + 6, y + 5, x + 8, y + 5);
    u8g2_DrawLine(d, x + 8, y + 5, x + 10, y + 7);

    u8g2_DrawDisc(d, x + 7, y + 10, 1, U8G2_DRAW_ALL);
}

// --- Page 1: Home (Fixed Spacing) ---
// --- Page 1: Home (Optimized for Spacing and Alignment) ---
// --- Page 1: Home (Fully Re-aligned for 128x64 ST7920) ---
// --- Re-added Thermometer and Gauge Icons ---
// ============================================================
// RE-DEFINED ICON HELPERS
// ============================================================

static void draw_vibration_icon(u8g2_t *d, int x, int y)
{
    u8g2_DrawLine(d, x, y + 4, x + 2, y + 4);
    u8g2_DrawLine(d, x + 2, y + 4, x + 3, y + 2);
    u8g2_DrawLine(d, x + 3, y + 2, x + 5, y + 6);
    u8g2_DrawLine(d, x + 5, y + 6, x + 7, y + 1);
    u8g2_DrawLine(d, x + 7, y + 1, x + 9, y + 4);
}

/* One complete 0..2pi sine wave, sized to match the vibration icon. */
static void draw_current_wave_icon(u8g2_t *d, int x, int y)
{
    u8g2_DrawLine(d, x, y + 4, x + 1, y + 2);
    u8g2_DrawLine(d, x + 1, y + 2, x + 2, y + 1);
    u8g2_DrawHLine(d, x + 2, y + 1, 2);
    u8g2_DrawLine(d, x + 3, y + 1, x + 4, y + 2);
    u8g2_DrawLine(d, x + 4, y + 2, x + 5, y + 4);
    u8g2_DrawLine(d, x + 5, y + 4, x + 6, y + 6);
    u8g2_DrawLine(d, x + 6, y + 6, x + 7, y + 7);
    u8g2_DrawHLine(d, x + 7, y + 7, 2);
    u8g2_DrawLine(d, x + 8, y + 7, x + 9, y + 6);
    u8g2_DrawLine(d, x + 9, y + 6, x + 10, y + 4);
}

static int draw_styled_chunk(u8g2_t *d, int x, int y, const char *text,
                             size_t length, bool bold)
{
    char chunk[64];
    if (length >= sizeof(chunk))
        length = sizeof(chunk) - 1;
    memcpy(chunk, text, length);
    chunk[length] = '\0';
    u8g2_DrawStr(d, x, y, chunk);
    if (bold)
        u8g2_DrawStr(d, x + 1, y, chunk);
    return u8g2_GetStrWidth(d, chunk);
}

static void draw_styled_str(u8g2_t *d, int x, int y, const char *text,
                            bool bold)
{
    if (!text)
        return;

    const char *p = text;
    while (*p)
    {
        const char *energy = strstr(p, "ENERGYMETER");
        const char *down = strstr(p, "v CLEAR");
        const char *special = NULL;
        if (energy && down)
            special = (energy < down) ? energy : down;
        else
            special = energy ? energy : down;

        if (!special)
        {
            (void)draw_styled_chunk(d, x, y, p, strlen(p), bold);
            break;
        }

        if (special > p)
            x += draw_styled_chunk(d, x, y, p, (size_t)(special - p), bold);

        if (special == energy)
        {
            int energy_w = draw_styled_chunk(d, x, y, "ENERGY", 6, bold);
            /* One genuinely blank pixel between the two displayed words. */
            x += energy_w + (bold ? 2 : 1);
            x += draw_styled_chunk(d, x, y, "METER", 5, bold);
            p = special + strlen("ENERGYMETER");
        }
        else
        {
            /* Draw a font-independent down chevron instead of lowercase v. */
            u8g2_DrawLine(d, x, y - 3, x + 2, y - 1);
            u8g2_DrawLine(d, x + 2, y - 1, x + 4, y - 3);
            /* Symbol ends at x+4; x+5 through x+10 remain blank. */
            x += 11;
            p = special + 2; /* Skip the original "v ". */
        }
    }
}

static void draw_bold_str(u8g2_t *d, int x, int y, const char *text)
{
    draw_styled_str(d, x, y, text, true);
}

static void draw_page_str(u8g2_t *d, int x, int y, const char *text)
{
    draw_styled_str(d, x, y, text, false);
}

static void draw_energy_icon(u8g2_t *d, int x, int y)
{
    u8g2_DrawTriangle(d, x + 7, y, x + 2, y + 7, x + 6, y + 7);
    u8g2_DrawTriangle(d, x + 6, y + 5, x + 11, y + 5, x + 4, y + 12);
}

static void draw_ac_current_icon(u8g2_t *d, int x, int y)
{
    u8g2_DrawHLine(d, x, y + 1, 3);
    u8g2_DrawVLine(d, x + 1, y + 1, 9);
    u8g2_DrawHLine(d, x, y + 10, 3);
    u8g2_DrawLine(d, x + 5, y + 7, x + 7, y + 4);
    u8g2_DrawLine(d, x + 7, y + 4, x + 9, y + 4);
    u8g2_DrawLine(d, x + 9, y + 4, x + 12, y + 8);
    u8g2_DrawLine(d, x + 12, y + 8, x + 14, y + 8);
}

static void draw_encoder_icon(u8g2_t *d, int x, int y)
{
    u8g2_DrawCircle(d, x + 6, y + 6, 5, U8G2_DRAW_ALL);
    u8g2_DrawCircle(d, x + 6, y + 6, 2, U8G2_DRAW_ALL);
    u8g2_DrawLine(d, x + 6, y + 1, x + 6, y + 3);
    u8g2_DrawLine(d, x + 6, y + 9, x + 6, y + 11);
    u8g2_DrawLine(d, x + 1, y + 6, x + 3, y + 6);
    u8g2_DrawLine(d, x + 9, y + 6, x + 11, y + 6);
}

static void draw_proxy_icon(u8g2_t *d, int x, int y)
{
    u8g2_DrawFrame(d, x, y + 3, 5, 7);
    u8g2_DrawDisc(d, x + 2, y + 6, 1, U8G2_DRAW_ALL);
    u8g2_DrawLine(d, x + 7, y + 3, x + 10, y + 1);
    u8g2_DrawLine(d, x + 7, y + 6, x + 11, y + 6);
    u8g2_DrawLine(d, x + 7, y + 9, x + 10, y + 11);
}

static void draw_proxy_count_value(u8g2_t *d, int x, int y, uint32_t count)
{
    char buf[12];
    snprintf(buf, sizeof(buf), "%lu", (unsigned long)count);

    u8g2_SetFont(d, u8g2_font_6x12_tf);
    int w = u8g2_GetStrWidth(d, buf);
    if (w > 39)
    {
        u8g2_SetFont(d, u8g2_font_5x7_tf);
        w = u8g2_GetStrWidth(d, buf);
    }
    if (w > 39)
    {
        u8g2_SetFont(d, u8g2_font_4x6_tf);
        w = u8g2_GetStrWidth(d, buf);
    }

    int draw_x = x + ((39 - w) / 2);
    if (draw_x < x)
        draw_x = x;
    draw_page_str(d, draw_x, y, buf);
}

static void draw_page_number(u8g2_t *d, const datameter_page_meta_t *meta)
{
    char page_text[8];
    uint8_t index = (meta && meta->page_index) ? meta->page_index : 1;
    uint8_t count = (meta && meta->page_count) ? meta->page_count : 1;
    snprintf(page_text, sizeof(page_text), "%u/%u", index, count);
    draw_page_str(d, 126 - u8g2_GetStrWidth(d, page_text), 10, page_text);
}

static void draw_info_footer(u8g2_t *d, const datameter_page_meta_t *meta)
{
    u8g2_DrawHLine(d, 0, 54, 128);
    u8g2_SetFont(d, u8g2_font_5x7_tf);

    if (meta && meta->status_text && meta->status_text[0])
    {
        int w = u8g2_GetStrWidth(d, meta->status_text);
        draw_bold_str(d, (128 - w) / 2, 63, meta->status_text);
        return;
    }

    bool has_prev = meta ? meta->has_prev : false;
    bool has_next = meta ? meta->has_next : false;
    if (has_prev)
    {
        draw_page_str(d, 2, 63, "< PREV");
    }
    else
    {
        draw_page_str(d, 2, 63, "B HOME");
    }
    if (has_prev && has_next)
    {
        draw_page_str(d, 52, 63, "B HOME");
    }
    if (has_next)
    {
        const char *next_hint = "NEXT >";
        draw_page_str(d, 126 - u8g2_GetStrWidth(d, next_hint), 63,
                      next_hint);
    }
    else if (has_prev)
    {
        const char *home_hint = "B HOME";
        draw_page_str(d, 126 - u8g2_GetStrWidth(d, home_hint), 63,
                      home_hint);
    }
}

static void format_float_or_dash(char *buf, size_t len, const char *fmt,
                                 float value)
{
    if (isfinite(value))
    {
        snprintf(buf, len, fmt, value);
    }
    else
    {
        snprintf(buf, len, "--");
    }
}

static void format_current_or_dash(char *buf, size_t len, float value,
                                   bool ct_open)
{
    if (!isfinite(value))
    {
        snprintf(buf, len, "--");
    }
    else if (ct_open)
    {
        snprintf(buf, len, "OPEN");
    }
    else
    {
        if (value < 0.0f)
            value = 0.0f;
        if (value < 100.0f)
        {
            snprintf(buf, len, "%.2f", value);
        }
        else
        {
            snprintf(buf, len, "%.0f", value);
        }
    }
}

static const uint8_t STARTUP_LOGO_BMP[] = {
    0x7F, 0xC0, 0x00, 0x00, 0x00, 0x00,
    0xFF, 0xF0, 0x07, 0x00, 0x00, 0x00,
    0xFF, 0xF8, 0x07, 0xE0, 0x00, 0x00,
    0xFF, 0xFC, 0x07, 0xF8, 0x00, 0x00,
    0xFF, 0xFE, 0x07, 0xFC, 0x00, 0x00,
    0xFF, 0xFF, 0x07, 0xFE, 0x00, 0x00,
    0xFF, 0xFF, 0x07, 0xFF, 0x00, 0x00,
    0xFF, 0xFF, 0x87, 0xFF, 0x80, 0x00,
    0xFF, 0xFF, 0x87, 0xFF, 0x80, 0x00,
    0xFF, 0xFF, 0x83, 0xFF, 0xC0, 0x00,
    0xFF, 0xFF, 0x80, 0xFF, 0xC0, 0x00,
    0xFF, 0xFF, 0x9F, 0x7F, 0xC0, 0x00,
    0xFF, 0xFF, 0xBF, 0xBF, 0xE0, 0x00,
    0xFF, 0xFF, 0xBF, 0xBF, 0xE0, 0x00,
    0xFF, 0xFF, 0xBF, 0xB7, 0xC0, 0x00,
    0xFF, 0xFF, 0xBF, 0x80, 0x00, 0x00,
    0xFF, 0xFF, 0x9F, 0x00, 0x00, 0x00,
    0xFF, 0xFF, 0x8E, 0x00, 0x00, 0x00,
    0xFF, 0xFF, 0x9F, 0x00, 0x00, 0x00,
    0xFF, 0xFF, 0xBF, 0x80, 0x00, 0x00,
    0xFF, 0xFF, 0xBF, 0x80, 0x00, 0x00,
    0xFF, 0xFF, 0xBF, 0x80, 0x00, 0x00,
    0xFF, 0xFF, 0xBF, 0x80, 0x00, 0x00,
    0xFF, 0xFF, 0xFF, 0x80, 0x00, 0x00,
    0xFF, 0xFF, 0xFF, 0x9F, 0xF8, 0x00,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFC, 0x00,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFE, 0x00,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x80,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x80,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xC0,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x80,
    0xFF, 0xFF, 0xFE, 0xFF, 0xFF, 0xC0,
    0x7F, 0xFF, 0xFC, 0xFF, 0xFF, 0xC0,
    0x7F, 0xFF, 0xFC, 0xFF, 0xFF, 0xC0,
    0x3F, 0xFF, 0xFC, 0xFF, 0xFF, 0xC0,
    0x1F, 0xFF, 0xFC, 0x7F, 0xFF, 0xC0,
    0x0F, 0xFF, 0xFC, 0x3F, 0xFF, 0x80,
    0x07, 0xFF, 0xFC, 0x1F, 0xFF, 0xC0,
    0x01, 0xFF, 0xFC, 0x07, 0xFF, 0x80};

#define STARTUP_LOGO_W 48
#define STARTUP_LOGO_H 40
#define STARTUP_LOGO_STRIDE 6

static void draw_startup_logo_bitmap(u8g2_t *d, int x, int y,
                                     uint8_t fade_step)
{
    for (int row = 0; row < STARTUP_LOGO_H; row++)
    {
        for (int col = 0; col < STARTUP_LOGO_W; col++)
        {
            uint8_t byte = STARTUP_LOGO_BMP[row * STARTUP_LOGO_STRIDE + (col / 8)];
            bool on = (byte & (0x80u >> (col & 7))) != 0;
            if (!on)
            {
                continue;
            }
            if (fade_step < 4 && ((row + col) & 0x03) > fade_step)
            {
                continue;
            }
            u8g2_DrawPixel(d, x + col, y + row);
        }
    }
}

static void draw_startup_logo_animation_impl(u8g2_t *d)
{
    if (!d)
        return;

    const int x = (128 - STARTUP_LOGO_W) / 2;
    const int y = (64 - STARTUP_LOGO_H) / 2;

    u8g2_ClearBuffer(d);
    lcd_submit_frame(d);
    vTaskDelay(pdMS_TO_TICKS(3000));

    for (uint8_t step = 0; step <= 4; step++)
    {
        u8g2_ClearBuffer(d);
        draw_startup_logo_bitmap(d, x, y, step);
        lcd_submit_frame(d);
        vTaskDelay(pdMS_TO_TICKS(220));
    }

    vTaskDelay(pdMS_TO_TICKS(350));
}

static void draw_boot_identity_page_impl(u8g2_t *d, const char *device_id,
                             const char *boot_reason)
{
    const char *id = (device_id && device_id[0]) ? device_id : "UNKNOWN";
    const char *reason =
        (boot_reason && boot_reason[0]) ? boot_reason : "UNKNOWN";
    char reason_line[32];

    u8g2_ClearBuffer(d);

    /* Prefer a prominent ID, but step down for longer configured names. */
    u8g2_SetFont(d, u8g2_font_helvB14_tr);
    if (u8g2_GetStrWidth(d, id) > 124) {
        u8g2_SetFont(d, u8g2_font_helvB10_tr);
    }
    if (u8g2_GetStrWidth(d, id) > 124) {
        u8g2_SetFont(d, u8g2_font_helvB08_tr);
    }
    int id_x = (128 - u8g2_GetStrWidth(d, id)) / 2;
    if (id_x < 2) id_x = 2;
    u8g2_DrawStr(d, id_x, 29, id);

    snprintf(reason_line, sizeof(reason_line), "BOOT REASON: %s", reason);
    u8g2_SetFont(d, u8g2_font_4x6_tr);
    int reason_x = (128 - u8g2_GetStrWidth(d, reason_line)) / 2;
    if (reason_x < 2) reason_x = 2;
    u8g2_DrawStr(d, reason_x, 47, reason_line);

    lcd_submit_frame(d);
}

static void draw_home_reason_code(u8g2_t *d, int x, int label_y,
                                  const datameter_home_t *m)
{
    const char *code = (m->reason_code[0] != '\0') ? m->reason_code : "__";
    u8g2_SetFont(d, u8g2_font_5x7_tf);
    draw_page_str(d, x, label_y, "CODE");

    if (m->reason_ok)
    {
        draw_bold_str(d, x + 30, label_y, "OK");
    }
    else
    {
        draw_bold_str(d, x + 30, label_y, code);
    }
}

// ============================================================
// FINAL HOME PAGE WITH ALL ELEMENTS
// ============================================================

static void draw_home_page_impl(u8g2_t *d, const datameter_home_t *m)
{
    char buf[32];
    char production_number[16];
    const char *state_text =
        (m->state == MACHINE_RUN) ? "RUN" : (m->state == MACHINE_STOP) ? "STP"
                                                                       : "IDLE";
    u8g2_ClearBuffer(d);
    u8g2_SetFontMode(d, 1);

    // 1. TOP BAR
    u8g2_SetFont(d, u8g2_font_6x10_tf);
    u8g2_DrawStr(d, 2, 9, m->time_text);
    /* Keep the identity centred in the space left by the actual connectivity
     * content. LTE occupies more width than the Wi-Fi icon, so its divider
     * and identity centre move left together. */
    const int header_left_divider_x = 33;
    int network_label_w = 0;
    int network_label_right = m->mqtt_connected ? 98 : 104;
    int connectivity_left = 100; /* Wi-Fi icon begins at x=100. */
    if (m->network_label[0] != '\0')
    {
        network_label_w = u8g2_GetStrWidth(d, m->network_label);
        connectivity_left = network_label_right - network_label_w;
    }
    const int header_right_divider_x = connectivity_left - 3;
    u8g2_DrawVLine(d, header_left_divider_x, 1, 9);
    u8g2_DrawVLine(d, header_right_divider_x, 1, 9);
    if (m->network_label[0] != '\0')
    {
        u8g2_DrawStr(d, network_label_right - network_label_w, 9,
                     m->network_label);
        if (m->mqtt_connected)
        {
            u8g2_SetFont(d, u8g2_font_5x7_tf);
            u8g2_DrawStr(d, 100, 9, "M");
            u8g2_SetFont(d, u8g2_font_6x10_tf);
        }
        draw_signal_icon(d, 108, 0, m->signal_level);
    }
    else
    {
        draw_wifi_icon(d, 100, 0, m->signal_level);
        if (m->mqtt_connected)
        {
            u8g2_SetFont(d, u8g2_font_5x7_tf);
            u8g2_DrawStr(d, 119, 9, "M");
            u8g2_SetFont(d, u8g2_font_6x10_tf);
        }
    }

    if (m->device_id[0] != '\0')
    {
        if (strncmp(m->device_id, "DM-", 3) == 0 && m->device_id[3] != '\0')
        {
            /* Header-only presentation: DM-117 becomes DM 117. Draw each
             * section separately for exact D/M and M/number pixel spacing. */
            const char *number = m->device_id + 3;
            u8g2_SetFont(d, u8g2_font_5x7_tf);
            int d_w = u8g2_GetStrWidth(d, "D");
            int m_w = u8g2_GetStrWidth(d, "M");
            int number_w = u8g2_GetStrWidth(d, number);
            /* Bold drawing extends every glyph one pixel to the right. Use a
             * two-pixel advance after D to leave one visible blank column. */
            int total_w = d_w + 2 + m_w + 3 + number_w + 1;
            int identity_left = header_left_divider_x + 1;
            int identity_width = header_right_divider_x - identity_left;
            int x = identity_left + (identity_width - total_w) / 2;
            draw_bold_str(d, x, 9, "D");
            x += d_w + 2;
            draw_bold_str(d, x, 9, "M");
            x += m_w + 3;
            draw_bold_str(d, x, 9, number);
        }
        else
        {
            /* Preserve non-standard configured IDs rather than hiding them. */
            u8g2_SetFont(d, u8g2_font_4x6_tf);
            int device_id_w = u8g2_GetStrWidth(d, m->device_id) + 1;
            int identity_left = header_left_divider_x + 1;
            int identity_width = header_right_divider_x - identity_left;
            draw_bold_str(d,
                          identity_left + (identity_width - device_id_w) / 2,
                          9, m->device_id);
        }
    }
    u8g2_DrawHLine(d, 0, 11, 128);

    // 2. Large state and the production/power panel.
    u8g2_SetFont(d, u8g2_font_fub17_tr);
    u8g2_DrawStr(d, 2, 32, state_text);

    // The main production value uses the same Helvetica family as kW, one
    // size larger. Removing the heading lets both rows move upward.
    bool production_enabled = strcmp(m->production_value, "DISABLED") != 0;
    snprintf(production_number, sizeof(production_number), "%s",
             m->production_value);
    char *production_unit = strrchr(production_number, ' ');
    if (production_unit != NULL)
        *production_unit++ = '\0';
    u8g2_SetFont(d, u8g2_font_helvR10_tr);
    int production_number_w = u8g2_GetStrWidth(d, production_number);
    int production_unit_w = 0;
    const uint8_t *production_unit_font = u8g2_font_helvR08_tr;
    if (production_unit != NULL)
    {
        u8g2_SetFont(d, production_unit_font);
        production_unit_w = u8g2_GetStrWidth(d, production_unit);
    }
    int production_x = 126 - production_number_w -
                       (production_unit ? 3 + production_unit_w : 0);
    if (production_enabled)
    {
        u8g2_SetFont(d, u8g2_font_helvR10_tr);
        u8g2_DrawStr(d, production_x, 26, production_number);
        if (production_unit != NULL)
        {
            u8g2_SetFont(d, production_unit_font);
            u8g2_DrawStr(d, production_x + production_number_w + 3, 26,
                         production_unit);
        }
    }

    snprintf(buf, sizeof(buf), "%.2f", m->power_value);
    const uint8_t *power_number_font = production_enabled
                                            ? u8g2_font_helvR08_tr
                                            : u8g2_font_helvR10_tr;
    u8g2_SetFont(d, power_number_font);
    int power_number_w = u8g2_GetStrWidth(d, buf);
    u8g2_SetFont(d, u8g2_font_5x7_tf);
    int power_unit_w = u8g2_GetStrWidth(d, "kW");
    int power_x = 126 - power_number_w - 3 - power_unit_w;
    u8g2_SetFont(d, power_number_font);
    int power_y = production_enabled ? 40 : 26;
    u8g2_DrawStr(d, power_x, power_y, buf);
    u8g2_SetFont(d, u8g2_font_5x7_tf);
    u8g2_DrawStr(d, power_x + power_number_w + 3, power_y, "kW");

    /* Transient system/OTA messages take priority over the IDLE reason-code
     * prompt so important outcomes are visible to the operator. */
    if (m->status_text[0] != '\0')
    {
        u8g2_SetFont(d, u8g2_font_4x6_tf);
        int status_w = u8g2_GetStrWidth(d, m->status_text);
        u8g2_DrawStr(d, (128 - status_w) / 2, 49, m->status_text);
    }
    else if (m->reason_prompt)
    {
        u8g2_SetFont(d, u8g2_font_5x7_tf);
        int reason_x = 2;
        u8g2_DrawStr(d, reason_x, 48, "ENTER");
        reason_x += u8g2_GetStrWidth(d, "ENTER") + 4;
        u8g2_DrawStr(d, reason_x, 48, "REASON");
        reason_x += u8g2_GetStrWidth(d, "REASON") + 4;
        u8g2_DrawStr(d, reason_x, 48, "CODE");
    }
    else
    {
        if (m->reason_visible)
        {
            draw_home_reason_code(d, 2, 44, m);
        }
    }

    // IDLE alone keeps the navigation footer used for reason-code operation.
    if (m->state == MACHINE_IDLE)
    {
        u8g2_DrawHLine(d, 0, 51, 128);
        u8g2_SetFont(d, u8g2_font_5x7_tf);
        u8g2_DrawStr(d, 2, 61, "B INFO");
        const char *menu_hint = "E MENU";
        u8g2_DrawStr(d, 126 - u8g2_GetStrWidth(d, menu_hint), 61,
                     menu_hint);
        lcd_submit_frame(d);
        return;
    }

    // RUN and STOP use a two-cell live sensor footer.
    u8g2_DrawHLine(d, 0, 51, 128);
    u8g2_SetFont(d, u8g2_font_5x7_tf);
    draw_vibration_icon(d, 3, 54);
    if (isfinite(m->vibration_mm_s))
        snprintf(buf, sizeof(buf), "%.1fmm/s", m->vibration_mm_s);
    else
        snprintf(buf, sizeof(buf), "--mm/s");
    u8g2_DrawStr(d, 16, 62, buf);

    if (isfinite(m->current_a))
        snprintf(buf, sizeof(buf), "%.2fA", m->current_a);
    else
        snprintf(buf, sizeof(buf), "--A");
    int current_w = u8g2_GetStrWidth(d, buf);
    int current_group_x = 126 - (11 + 3 + current_w);
    draw_current_wave_icon(d, current_group_x, 54);
    u8g2_DrawStr(d, current_group_x + 14, 62, buf);

    lcd_submit_frame(d);
}
// --- Page 2: Energy (Fixed Overlap) ---
// --- Page 2: Energy (Updated with kWh units) ---
static void draw_energy_page_impl(u8g2_t *d, const datameter_energy_t *e)
{
    char buf[20];
    const char *unit = "kWh";
    u8g2_ClearBuffer(d);

    // 1. Header (Verbatim from image_ceb200.png)
    u8g2_SetFont(d, u8g2_font_6x12_tf);
    draw_energy_icon(d, 3, 1);
    draw_page_str(d, 20, 10, "ENERGY");
    draw_page_number(d, &e->meta);
    u8g2_DrawHLine(d, 0, 12, 128);

    // 2. Grid
    u8g2_DrawVLine(d, 64, 12, 42); // Vertical center divider
    u8g2_DrawHLine(d, 0, 33, 128); // Horizontal middle divider

    u8g2_SetFont(d, u8g2_font_5x7_tf); // Label font

    // --- TOP LEFT: ACTIVE ---
    draw_page_str(d, 4, 21, "ACTIVE");
    u8g2_SetFont(d, u8g2_font_7x13_tf);
    snprintf(buf, sizeof(buf), "%.2f", e->active_kwh);
    draw_page_str(d, 4, 31, buf);
    int active_w = u8g2_GetStrWidth(d, buf);
    u8g2_SetFont(d, u8g2_font_4x6_tf); // Tiny font for unit
    draw_page_str(d, 4 + active_w + 3, 31, unit);

    // --- TOP RIGHT: STANDBY ---
    u8g2_SetFont(d, u8g2_font_5x7_tf);
    draw_page_str(d, 68, 21, "STANDBY");
    u8g2_SetFont(d, u8g2_font_7x13_tf);
    snprintf(buf, sizeof(buf), "%.2f", e->standby_kwh);
    draw_page_str(d, 68, 31, buf);
    int standby_w = u8g2_GetStrWidth(d, buf);
    u8g2_SetFont(d, u8g2_font_4x6_tf);
    draw_page_str(d, 68 + standby_w + 3, 31, unit);

    // --- BOTTOM LEFT: TODAY ---
    u8g2_SetFont(d, u8g2_font_5x7_tf);
    draw_page_str(d, 4, 42, "TODAY");
    u8g2_SetFont(d, u8g2_font_7x13_tf);
    snprintf(buf, sizeof(buf), "%.2f", e->today_kwh);
    draw_page_str(d, 4, 52, buf);
    int today_w = u8g2_GetStrWidth(d, buf);
    u8g2_SetFont(d, u8g2_font_4x6_tf);
    draw_page_str(d, 4 + today_w + 3, 52, unit);

    // --- BOTTOM RIGHT: TOTAL ---
    u8g2_SetFont(d, u8g2_font_5x7_tf);
    draw_page_str(d, 68, 42, "TOTAL");
    u8g2_SetFont(d, u8g2_font_7x13_tf);
    snprintf(buf, sizeof(buf), "%.1f", e->total_kwh);
    draw_page_str(d, 68, 52, buf);
    int total_w = u8g2_GetStrWidth(d, buf);
    u8g2_SetFont(d, u8g2_font_4x6_tf);
    draw_page_str(d, 68 + total_w + 3, 52, unit);

    // 3. Footer
    draw_info_footer(d, &e->meta);

    lcd_submit_frame(d);
}

static void draw_speed_page_impl(u8g2_t *d, const datameter_speed_t *s)
{
    char buf[20];
    if (!d || !s)
    {
        return;
    }

    u8g2_ClearBuffer(d);

    u8g2_SetFont(d, u8g2_font_6x12_tf);
    draw_encoder_icon(d, 3, 1);
    draw_page_str(d, 20, 10, "SPEED");
    draw_page_number(d, &s->meta);
    u8g2_DrawHLine(d, 0, 12, 128);

    u8g2_DrawVLine(d, 64, 12, 42);
    u8g2_DrawHLine(d, 0, 33, 128);

    u8g2_SetFont(d, u8g2_font_5x7_tf);
    draw_page_str(d, 4, 21, "CURRENT");
    u8g2_SetFont(d, u8g2_font_7x13_tf);
    snprintf(buf, sizeof(buf), "%.2f", s->current_length_m);
    draw_page_str(d, 4, 31, buf);
    int current_w = u8g2_GetStrWidth(d, buf);
    u8g2_SetFont(d, u8g2_font_4x6_tf);
    draw_page_str(d, 4 + current_w + 3, 31, "m");

    u8g2_SetFont(d, u8g2_font_5x7_tf);
    draw_page_str(d, 68, 21, "TOTAL");
    u8g2_SetFont(d, u8g2_font_7x13_tf);
    snprintf(buf, sizeof(buf), "%.1f", s->total_length_m);
    draw_page_str(d, 68, 31, buf);
    int total_w = u8g2_GetStrWidth(d, buf);
    u8g2_SetFont(d, u8g2_font_4x6_tf);
    draw_page_str(d, 68 + total_w + 3, 31, "m");

    u8g2_SetFont(d, u8g2_font_5x7_tf);
    draw_page_str(d, 4, 42, "SPEED");
    u8g2_SetFont(d, u8g2_font_7x13_tf);
    snprintf(buf, sizeof(buf), "%.2f", s->speed_mps);
    draw_page_str(d, 4, 52, buf);
    int speed_w = u8g2_GetStrWidth(d, buf);
    u8g2_SetFont(d, u8g2_font_4x6_tf);
    draw_page_str(d, 4 + speed_w + 3, 52, "m/s");

    draw_info_footer(d, &s->meta);
    lcd_submit_frame(d);
}

static void draw_duration_page_impl(u8g2_t *d, const datameter_duration_t *p)
{
    char buf[24];
    if (!d || !p)
    {
        return;
    }

    u8g2_ClearBuffer(d);

    u8g2_SetFont(d, u8g2_font_6x12_tf);
    draw_energy_icon(d, 3, 1);
    draw_page_str(d, 20, 10, p->title[0] ? p->title : "CYCLE COUNT");
    draw_page_number(d, &p->meta);
    u8g2_DrawHLine(d, 0, 12, 128);

    if (!p->selected_valid)
    {
        u8g2_SetFont(d, u8g2_font_6x12_tf);
        draw_page_str(d, 15, 29, "NO PART SELECTED");
        u8g2_SetFont(d, u8g2_font_5x7_tf);
        draw_page_str(d, 27, 46, "SELECT EXISTING");
        draw_info_footer(d, &p->meta);
        lcd_submit_frame(d);
        return;
    }

    // The data grid is useful only when there is part data to separate.  Keep
    // the empty-state message clear of vertical and horizontal divider lines.
    u8g2_DrawVLine(d, 64, 12, 42);
    u8g2_DrawHLine(d, 0, 33, 128);

    u8g2_SetFont(d, u8g2_font_5x7_tf);
    draw_page_str(d, 4, 21, "COUNT");
    u8g2_SetFont(d, u8g2_font_7x13_tf);
    snprintf(buf, sizeof(buf), "%lu", (unsigned long)p->count);
    draw_page_str(d, 4, 31, buf);

    u8g2_SetFont(d, u8g2_font_5x7_tf);
    draw_page_str(d, 68, 21, "PART");
    u8g2_SetFont(d, u8g2_font_7x13_tf);
    snprintf(buf, sizeof(buf), "%lu", (unsigned long)p->part_number);
    draw_page_str(d, 68, 31, buf);

    u8g2_SetFont(d, u8g2_font_5x7_tf);
    draw_page_str(d, 4, 42, "CYCLE");
    u8g2_SetFont(d, u8g2_font_7x13_tf);
    snprintf(buf, sizeof(buf), "%02u:%02u",
             (unsigned)(p->cycle_time_s / 60),
             (unsigned)(p->cycle_time_s % 60));
    draw_page_str(d, 4, 52, buf);

    u8g2_SetFont(d, u8g2_font_5x7_tf);
    draw_page_str(d, 68, 42, "OPS");
    u8g2_SetFont(d, u8g2_font_7x13_tf);
    snprintf(buf, sizeof(buf), "%u/%u",
             (unsigned)p->operation_count,
             (unsigned)p->operations_per_part);
    draw_page_str(d, 68, 52, buf);

    draw_info_footer(d, &p->meta);
    lcd_submit_frame(d);
}

static void draw_count_page_impl(u8g2_t *d, const datameter_count_t *c)
{
    char buf[24];
    if (!d || !c)
    {
        return;
    }

    u8g2_ClearBuffer(d);

    u8g2_SetFont(d, u8g2_font_6x12_tf);
    draw_page_str(d, 20, 10, "COUNT");
    draw_page_number(d, &c->meta);
    u8g2_DrawHLine(d, 0, 12, 128);

    u8g2_SetFont(d, u8g2_font_5x7_tf);
    draw_bold_str(d, 4, 24, "CT A COUNT");
    u8g2_SetFont(d, u8g2_font_9x15_tf);
    snprintf(buf, sizeof(buf), "%lu", (unsigned long)c->count);
    draw_page_str(d, 4, 43, buf);

    u8g2_SetFont(d, u8g2_font_5x7_tf);
    draw_bold_str(d, 78, 24, c->enabled ? "ON" : "OFF");
    snprintf(buf, sizeof(buf), "A %.1f", c->current_a);
    draw_page_str(d, 78, 38, buf);
    snprintf(buf, sizeof(buf), "TH %.1f", c->threshold_a);
    draw_page_str(d, 78, 50, buf);

    draw_info_footer(d, &c->meta);

    lcd_submit_frame(d);
}

static void draw_vibration_page_impl(u8g2_t *d, const datameter_vibration_t *v)
{
    char buf[20];
    u8g2_ClearBuffer(d);

    u8g2_SetFont(d, u8g2_font_6x12_tf);
    draw_vibration_icon(d, 4, 2);
    draw_page_str(d, 20, 10, "VIBRATION");
    draw_page_number(d, &v->meta);
    u8g2_DrawHLine(d, 0, 12, 128);

    u8g2_DrawVLine(d, 64, 12, 42);
    u8g2_DrawHLine(d, 0, 33, 128);

    u8g2_SetFont(d, u8g2_font_5x7_tf);
    draw_page_str(d, 4, 21, "DISP");
    u8g2_SetFont(d, u8g2_font_7x13_tf);
    format_float_or_dash(buf, sizeof(buf), "%.1f", v->displacement_um);
    draw_page_str(d, 4, 31, buf);
    int disp_w = u8g2_GetStrWidth(d, buf);
    u8g2_SetFont(d, u8g2_font_4x6_tf);
    draw_page_str(d, 4 + disp_w + 3, 31, "um");

    u8g2_SetFont(d, u8g2_font_5x7_tf);
    draw_page_str(d, 68, 21, "ACCEL");
    u8g2_SetFont(d, u8g2_font_7x13_tf);
    format_float_or_dash(buf, sizeof(buf), "%.2f", v->acceleration_g);
    draw_page_str(d, 68, 31, buf);
    int acc_w = u8g2_GetStrWidth(d, buf);
    u8g2_SetFont(d, u8g2_font_4x6_tf);
    draw_page_str(d, 68 + acc_w + 3, 31, "g");

    u8g2_SetFont(d, u8g2_font_5x7_tf);
    draw_page_str(d, 4, 42, "VELOCITY");
    u8g2_SetFont(d, u8g2_font_7x13_tf);
    format_float_or_dash(buf, sizeof(buf), "%.1f", v->velocity_mm_s);
    draw_page_str(d, 4, 52, buf);
    int vel_w = u8g2_GetStrWidth(d, buf);
    u8g2_SetFont(d, u8g2_font_4x6_tf);
    draw_page_str(d, 4 + vel_w + 3, 52, "mm/s");

    u8g2_SetFont(d, u8g2_font_5x7_tf);
    draw_page_str(d, 68, 42, "FREQ");
    u8g2_SetFont(d, u8g2_font_7x13_tf);
    format_float_or_dash(buf, sizeof(buf), "%.1f", v->frequency_hz);
    draw_page_str(d, 68, 52, buf);
    int freq_w = u8g2_GetStrWidth(d, buf);
    u8g2_SetFont(d, u8g2_font_4x6_tf);
    draw_page_str(d, 68 + freq_w + 3, 52, "Hz");

    draw_info_footer(d, &v->meta);

    lcd_submit_frame(d);
}

static void draw_phase_page_impl(u8g2_t *d, const datameter_phase_t *p)
{
    char buf[20];
    u8g2_ClearBuffer(d);

    u8g2_SetFont(d, u8g2_font_6x12_tf);
    draw_ac_current_icon(d, 3, 0);
    draw_page_str(d, 20, 10, "PHASE");
    draw_page_number(d, &p->meta);
    u8g2_DrawHLine(d, 0, 12, 128);

    if (p->single_phase)
    {
        char phase = '-';
        float current = NAN;
        uint8_t open_bit = 0;

        if (isfinite(p->current_a))
        {
            phase = 'A';
            current = p->current_a;
            open_bit = 0x01;
        }
        else if (isfinite(p->current_b))
        {
            phase = 'B';
            current = p->current_b;
            open_bit = 0x02;
        }
        else if (isfinite(p->current_c))
        {
            phase = 'C';
            current = p->current_c;
            open_bit = 0x04;
        }

        u8g2_SetFont(d, u8g2_font_5x7_tf);
        if (phase == '-')
        {
            draw_bold_str(d, 8, 25, "ACTIVE PHASE");
            u8g2_SetFont(d, u8g2_font_7x13_tf);
            draw_page_str(d, 8, 42, "--");
        }
        else
        {
            snprintf(buf, sizeof(buf), "PHASE %c", phase);
            draw_bold_str(d, 8, 25, buf);
            u8g2_SetFont(d, u8g2_font_7x13_tf);
            format_current_or_dash(buf, sizeof(buf), current,
                                   (p->ct_open_mask & open_bit) != 0);
            draw_page_str(d, 8, 43, buf);
            int w = u8g2_GetStrWidth(d, buf);
            if (strcmp(buf, "--") != 0 && strcmp(buf, "OPEN") != 0)
            {
                u8g2_SetFont(d, u8g2_font_4x6_tf);
                draw_page_str(d, 8 + w + 3, 43, "A");
            }
        }

        draw_info_footer(d, &p->meta);
        lcd_submit_frame(d);
        return;
    }

    u8g2_DrawVLine(d, 42, 12, 42);
    u8g2_DrawVLine(d, 85, 12, 42);

    u8g2_SetFont(d, u8g2_font_5x7_tf);
    draw_bold_str(d, 4, 23, "A");
    u8g2_SetFont(d, u8g2_font_6x12_tf);
    format_current_or_dash(buf, sizeof(buf), p->current_a,
                           (p->ct_open_mask & 0x01) != 0);
    draw_page_str(d, 4, 37, buf);
    int a_w = u8g2_GetStrWidth(d, buf);
    if (strcmp(buf, "--") != 0 && strcmp(buf, "OPEN") != 0)
    {
        u8g2_SetFont(d, u8g2_font_4x6_tf);
        draw_page_str(d, 4 + a_w + 3, 37, "A");
    }

    u8g2_SetFont(d, u8g2_font_5x7_tf);
    draw_bold_str(d, 47, 23, "B");
    u8g2_SetFont(d, u8g2_font_6x12_tf);
    format_current_or_dash(buf, sizeof(buf), p->current_b,
                           (p->ct_open_mask & 0x02) != 0);
    draw_page_str(d, 47, 37, buf);
    int b_w = u8g2_GetStrWidth(d, buf);
    if (strcmp(buf, "--") != 0 && strcmp(buf, "OPEN") != 0)
    {
        u8g2_SetFont(d, u8g2_font_4x6_tf);
        draw_page_str(d, 47 + b_w + 3, 37, "A");
    }

    u8g2_SetFont(d, u8g2_font_5x7_tf);
    draw_bold_str(d, 90, 23, "C");
    u8g2_SetFont(d, u8g2_font_6x12_tf);
    format_current_or_dash(buf, sizeof(buf), p->current_c,
                           (p->ct_open_mask & 0x04) != 0);
    draw_page_str(d, 90, 37, buf);
    int c_w = u8g2_GetStrWidth(d, buf);
    if (strcmp(buf, "--") != 0 && strcmp(buf, "OPEN") != 0)
    {
        u8g2_SetFont(d, u8g2_font_4x6_tf);
        draw_page_str(d, 90 + c_w + 3, 37, "A");
    }

    draw_info_footer(d, &p->meta);

    lcd_submit_frame(d);
}

static void draw_encoder_page_impl(u8g2_t *d, const datameter_encoder_t *e)
{
    char buf[20];
    u8g2_ClearBuffer(d);

    u8g2_SetFont(d, u8g2_font_6x12_tf);
    draw_encoder_icon(d, 4, 0);
    draw_page_str(d, 20, 10, "ENCODER");
    draw_page_number(d, &e->meta);
    u8g2_DrawHLine(d, 0, 12, 128);

    u8g2_DrawVLine(d, 42, 12, 42);
    u8g2_DrawVLine(d, 85, 12, 42);

    u8g2_SetFont(d, u8g2_font_5x7_tf);
    draw_bold_str(d, 4, 23, "RPM");
    u8g2_SetFont(d, u8g2_font_6x12_tf);
    format_float_or_dash(buf, sizeof(buf), "%.1f", e->rpm);
    draw_page_str(d, 4, 37, buf);

    u8g2_SetFont(d, u8g2_font_5x7_tf);
    draw_bold_str(d, 47, 23, "MPM");
    u8g2_SetFont(d, u8g2_font_6x12_tf);
    format_float_or_dash(buf, sizeof(buf), "%.1f", e->mpm);
    draw_page_str(d, 47, 37, buf);
    if (strcmp(buf, "--") != 0)
    {
        u8g2_SetFont(d, u8g2_font_4x6_tf);
        draw_page_str(d, 47, 47, "m/min");
    }

    u8g2_SetFont(d, u8g2_font_5x7_tf);
    draw_bold_str(d, 90, 23, "LEN");
    u8g2_SetFont(d, u8g2_font_6x12_tf);
    format_float_or_dash(buf, sizeof(buf), "%.1f", e->length_m);
    draw_page_str(d, 90, 37, buf);
    if (strcmp(buf, "--") != 0)
    {
        u8g2_SetFont(d, u8g2_font_4x6_tf);
        draw_page_str(d, 90, 47, "m");
    }

    draw_info_footer(d, &e->meta);

    lcd_submit_frame(d);
}

static void draw_encoder_count_page_impl(u8g2_t *d, const datameter_encoder_count_t *e)
{
    char buf[24];
    if (!d || !e)
    {
        return;
    }

    u8g2_ClearBuffer(d);

    u8g2_SetFont(d, u8g2_font_6x12_tf);
    draw_encoder_icon(d, 4, 0);
    draw_page_str(d, 20, 10, "ENC COUNT");
    draw_page_number(d, &e->meta);
    u8g2_DrawHLine(d, 0, 12, 128);

    u8g2_DrawVLine(d, 42, 12, 42);
    u8g2_DrawVLine(d, 85, 12, 42);

    u8g2_SetFont(d, u8g2_font_5x7_tf);
    draw_bold_str(d, 3, 23, "LEN");
    u8g2_SetFont(d, u8g2_font_6x12_tf);
    format_float_or_dash(buf, sizeof(buf), "%.1f", e->current_length_m);
    draw_page_str(d, 2, 37, buf);
    if (strcmp(buf, "--") != 0)
    {
        u8g2_SetFont(d, u8g2_font_4x6_tf);
        draw_page_str(d, 2, 47, "m");
    }

    u8g2_SetFont(d, u8g2_font_5x7_tf);
    draw_bold_str(d, 47, 23, "GOOD");
    draw_proxy_count_value(d, 44, 39, e->good_count);

    u8g2_SetFont(d, u8g2_font_5x7_tf);
    draw_bold_str(d, 90, 23, "SCRAP");
    draw_proxy_count_value(d, 87, 39, e->scrap_count);

    draw_info_footer(d, &e->meta);

    lcd_submit_frame(d);
}

static void draw_proxy_page_impl(u8g2_t *d, const datameter_proxy_t *p)
{
    u8g2_ClearBuffer(d);

    u8g2_SetFont(d, u8g2_font_6x12_tf);
    draw_proxy_icon(d, 4, 0);
    draw_page_str(d, 20, 10, "PROXY");
    draw_page_number(d, &p->meta);
    u8g2_DrawHLine(d, 0, 12, 128);

    u8g2_DrawVLine(d, 42, 12, 42);
    u8g2_DrawVLine(d, 85, 12, 42);

    u8g2_SetFont(d, u8g2_font_5x7_tf);
    draw_bold_str(d, 4, 23, "P-A");
    draw_proxy_count_value(d, 1, 39, p->count_a);

    u8g2_SetFont(d, u8g2_font_5x7_tf);
    draw_bold_str(d, 47, 23, "P-B");
    draw_proxy_count_value(d, 44, 39, p->count_b);

    u8g2_SetFont(d, u8g2_font_5x7_tf);
    draw_bold_str(d, 90, 23, "TOT");
    draw_proxy_count_value(d, 87, 39, p->count_total);

    draw_info_footer(d, &p->meta);

    lcd_submit_frame(d);
}

static void draw_proxy_production_page_impl(u8g2_t *d,
                                const datameter_proxy_production_t *p)
{
    char buf[24];
    if (!d || !p)
    {
        return;
    }

    u8g2_ClearBuffer(d);

    u8g2_SetFont(d, u8g2_font_6x12_tf);
    draw_proxy_icon(d, 4, 0);
    draw_page_str(d, 20, 10, p->title[0] ? p->title : "PROXY PROD");
    draw_page_number(d, &p->meta);
    u8g2_DrawHLine(d, 0, 12, 128);

    u8g2_DrawVLine(d, p->scrap_enabled ? 42 : 63, 12, 42);
    if (p->scrap_enabled)
        u8g2_DrawVLine(d, 85, 12, 42);

    u8g2_SetFont(d, u8g2_font_5x7_tf);
    draw_bold_str(d, p->scrap_enabled ? 3 : 14, 23, "LEN");
    u8g2_SetFont(d, u8g2_font_6x12_tf);
    if (p->meter_mode)
    {
        format_float_or_dash(buf, sizeof(buf), "%.1f", p->total_length_m);
    }
    else
    {
        snprintf(buf, sizeof(buf), "--");
    }
    draw_page_str(d, p->scrap_enabled ? 2 : 13, 37, buf);
    if (p->meter_mode)
    {
        u8g2_SetFont(d, u8g2_font_4x6_tf);
        draw_page_str(d, p->scrap_enabled ? 2 : 13, 47, "m");
    }

    if (p->scrap_enabled)
    {
        u8g2_SetFont(d, u8g2_font_5x7_tf);
        draw_bold_str(d, 45, 23, "SCRAP");
        if (p->meter_mode)
        {
            draw_proxy_count_value(d, 44, 39, p->scrap_count);
        }
        else
        {
            u8g2_SetFont(d, u8g2_font_6x12_tf);
            draw_page_str(d, 47, 39, "--");
        }
    }

    u8g2_SetFont(d, u8g2_font_5x7_tf);
    draw_bold_str(d, p->scrap_enabled ? 90 : 82, 23, "GOOD");
    draw_proxy_count_value(d, p->scrap_enabled ? 87 : 78, 39, p->good_count);

    draw_info_footer(d, &p->meta);

    lcd_submit_frame(d);
}

static void draw_lte_diag_page_impl(u8g2_t *d, const datameter_lte_diag_t *lte)
{
    char buf[32];
    if (!d || !lte)
    {
        return;
    }

    u8g2_ClearBuffer(d);
    u8g2_SetFont(d, u8g2_font_6x12_tf);
    draw_page_str(d, 8, 10, "-- LTE DIAG --");
    draw_page_number(d, &lte->meta);
    u8g2_DrawHLine(d, 0, 12, 128);

    u8g2_SetFont(d, u8g2_font_6x10_tf);
    u8g2_DrawStr(d, 0, 21, "NET:");
    u8g2_DrawStr(d, 24, 21, lte->network_text[0] ? lte->network_text : "--");
    u8g2_DrawStr(d, 80, 21, "SIM:");
    u8g2_DrawStr(d, 104, 21, lte->sim_text[0] ? lte->sim_text : "--");

    u8g2_SetFont(d, u8g2_font_5x7_tf);
    u8g2_DrawStr(d, 0, 39, "BAND:");
    u8g2_DrawStr(d, 28, 39, lte->band_text[0] ? lte->band_text : "--");
    if (lte->rssi_dbm == INT16_MIN)
    {
        u8g2_DrawStr(d, 0, 31, "RSSI: --");
    }
    else
    {
        snprintf(buf, sizeof(buf), "RSSI:%ddBm", (int)lte->rssi_dbm);
        u8g2_DrawStr(d, 0, 31, buf);
    }

    u8g2_DrawStr(d, 0, 47, "IP:");
    u8g2_DrawStr(d, 14, 47, lte->ip_text[0] ? lte->ip_text : "--");
    u8g2_SetFont(d, u8g2_font_4x6_tf);
    u8g2_DrawStr(d, 0, 54, lte->mqtt_text[0] ? lte->mqtt_text : "MQTT: --");

    u8g2_DrawHLine(d, 0, 57, 128);
    u8g2_SetFont(d, u8g2_font_5x7_tf);
    if (lte->meta.status_text && lte->meta.status_text[0])
    {
        int w = u8g2_GetStrWidth(d, lte->meta.status_text);
        draw_bold_str(d, (128 - w) / 2, 63, lte->meta.status_text);
    }
    else
    {
        if (lte->meta.has_prev)
        {
            u8g2_DrawStr(d, 2, 63, "< PREV");
        }
        else
        {
            u8g2_DrawStr(d, 2, 63, "B HOME");
        }
        if (lte->meta.has_prev && lte->meta.has_next)
        {
            u8g2_DrawStr(d, 47, 63, "B HOME");
        }
        if (lte->meta.has_next)
        {
            u8g2_DrawStr(d, 91, 63, "> NEXT");
        }
        else if (lte->meta.has_prev)
        {
            u8g2_DrawStr(d, 88, 63, "B HOME");
        }
    }
    lcd_submit_frame(d);
}

static void draw_wifi_diag_page_impl(u8g2_t *d, const datameter_wifi_diag_t *wifi)
{
    if (!d || !wifi)
        return;

    char buf[32];
    u8g2_ClearBuffer(d);
    u8g2_SetFont(d, u8g2_font_6x12_tf);
    draw_page_str(d, 8, 10, "-- WIFI DIAG --");
    draw_page_number(d, &wifi->meta);
    u8g2_DrawHLine(d, 0, 12, 128);

    u8g2_SetFont(d, u8g2_font_4x6_tf);
    snprintf(buf, sizeof(buf), "SSID:%.26s",
             wifi->ssid_text[0] ? wifi->ssid_text : "--");
    u8g2_DrawStr(d, 0, 21, buf);

    u8g2_SetFont(d, u8g2_font_5x7_tf);
    snprintf(buf, sizeof(buf), "RSSI:%ddBm", (int)wifi->rssi_dbm);
    u8g2_DrawStr(d, 0, 31, buf);
    snprintf(buf, sizeof(buf), "IP:%s",
             wifi->ip_text[0] ? wifi->ip_text : "--");
    u8g2_DrawStr(d, 0, 39, buf);
    snprintf(buf, sizeof(buf), "WIFI:%s",
             wifi->wifi_text[0] ? wifi->wifi_text : "--");
    u8g2_DrawStr(d, 0, 47, buf);
    snprintf(buf, sizeof(buf), "MQTT:%.15s %.10s",
             wifi->mqtt_text[0] ? wifi->mqtt_text : "--",
             wifi->mode_text[0] ? wifi->mode_text : "");
    u8g2_DrawStr(d, 0, 55, buf);

    draw_info_footer(d, &wifi->meta);
    lcd_submit_frame(d);
}

static void draw_text_page_internal(u8g2_t *d, const char *title,
                                    const char *line0, const char *line1,
                                    const char *line2, const char *line3,
                                    bool bold_title)
{
    if (!d)
        return;

    const char *lines[] = {line0, line1, line2, line3};
    u8g2_ClearBuffer(d);
    u8g2_SetFontMode(d, 1);
    u8g2_SetFont(d, u8g2_font_6x12_tf);
    if (bold_title)
    {
        draw_bold_str(d, 2, 10, title ? title : "STATUS");
    }
    else
    {
        draw_page_str(d, 2, 10, title ? title : "STATUS");
    }
    u8g2_DrawHLine(d, 0, 13, 128);

    u8g2_SetFont(d, u8g2_font_5x7_tf);
    int y = 25;
    for (int i = 0; i < 4; i++)
    {
        if (lines[i] && lines[i][0])
        {
            draw_page_str(d, 4, y, lines[i]);
        }
        y += 9;
    }

    lcd_submit_frame(d);
}

static void draw_text_page_impl(u8g2_t *d, const char *title, const char *line0,
                    const char *line1, const char *line2,
                    const char *line3)
{
    draw_text_page_internal(d, title, line0, line1, line2, line3, true);
}

static void draw_text_page_normal_impl(u8g2_t *d, const char *title, const char *line0,
                           const char *line1, const char *line2,
                           const char *line3)
{
    draw_text_page_internal(d, title, line0, line1, line2, line3, false);
}

static void draw_reason_search_page_impl(u8g2_t *d, const char *search_text,
                             const char *const *items, uint8_t item_count,
                             const char *status_text)
{
    if (!d)
        return;

    char search_bar[18];
    const char *query = (search_text && search_text[0]) ? search_text : "__";
    snprintf(search_bar, sizeof(search_bar), "SEARCH %s", query);

    u8g2_ClearBuffer(d);
    u8g2_SetFont(d, u8g2_font_6x12_tf);
    draw_page_str(d, 2, 10, "REASON CODE");
    u8g2_DrawHLine(d, 0, 13, 128);

    u8g2_DrawFrame(d, 2, 16, 124, 12);
    u8g2_SetFont(d, u8g2_font_5x7_tf);
    draw_bold_str(d, 6, 25, search_bar);

    if (status_text && status_text[0])
    {
        draw_bold_str(d, 6, 38, status_text);
    }
    else if (items && item_count > 0)
    {
        int y = 37;
        for (uint8_t i = 0; i < item_count && i < 3; i++)
        {
            draw_page_str(d, 6, y, items[i]);
            y += 8;
        }
    }
    else
    {
        draw_page_str(d, 6, 38, "ENTER FIRST DIGIT");
    }

    u8g2_DrawHLine(d, 0, 54, 128);
    draw_page_str(d, 2, 63, "B BACK");
    draw_page_str(d, 82, 63, "E OK");
    lcd_submit_frame(d);
}

// Helper for the Menu List Icon
static void draw_menu_icon(u8g2_t *d, int x, int y)
{
    u8g2_DrawHLine(d, x, y, 3);
    u8g2_DrawHLine(d, x + 4, y, 8);
    u8g2_DrawHLine(d, x, y + 3, 3);
    u8g2_DrawHLine(d, x + 4, y + 3, 8);
    u8g2_DrawHLine(d, x, y + 6, 3);
    u8g2_DrawHLine(d, x + 4, y + 6, 8);
}

static void draw_menu_list_page_ex_impl(u8g2_t *d, const char *title,
                            const char *page_text,
                            const char *const *items, uint8_t item_count,
                            uint8_t selected_index, bool bold,
                            bool show_footer)
{
    if (!d || !items || item_count == 0)
        return;

    char auto_page_text[8];
    const char *header_page_text = page_text;
    if (!header_page_text)
    {
        unsigned current =
            (selected_index < item_count) ? (unsigned)(selected_index + 1) : 1u;
        snprintf(auto_page_text, sizeof(auto_page_text), "%u/%u", current,
                 (unsigned)item_count);
        header_page_text = auto_page_text;
    }

    u8g2_ClearBuffer(d);

    /* Use the same compact visual grid as the home page: a 5x7 header at
     * baseline 9, separator at y=11, and footer separator at y=51. */
    draw_menu_icon(d, 2, 1);
    u8g2_SetFont(d, u8g2_font_5x7_tf);
    const char *header_title = title ? title : "MENU";
    /* Every bold menu heading uses the same explicit one-pixel letter
     * spacing. Bold rendering itself occupies one extra horizontal pixel,
     * so advance two pixels beyond the reported glyph width. */
    int title_x = 18;
    for (const char *p = header_title; *p; ++p)
    {
        char glyph[2] = {*p, '\0'};
        draw_bold_str(d, title_x, 9, glyph);
        title_x += u8g2_GetStrWidth(d, glyph) + 2;
    }
    int page_x = 126 - u8g2_GetStrWidth(d, header_page_text);
    u8g2_DrawStr(d, page_x, 9, header_page_text);
    u8g2_DrawHLine(d, 0, 11, 128);

    // 2. Menu Items
    u8g2_SetFont(d, u8g2_font_5x8_tf);
    bool has_selection = selected_index < item_count;
    int y_pos = 22;
    const int row_step = 12;
    const int box_height = 11;
    for (int i = 0; i < item_count && i < 3; i++)
    {
        if (has_selection && i == selected_index)
        {
            // Draw Highlight Box
            u8g2_SetDrawColor(d, 1);
            u8g2_DrawBox(d, 2, y_pos - 9, 124, box_height);

            // Draw Selection Arrow
            u8g2_SetDrawColor(d, 0); // Inverted text/arrow
            if (bold)
            {
                draw_bold_str(d, 5, y_pos, ">");
                draw_bold_str(d, 14, y_pos, items[i]);
            }
            else
            {
                draw_page_str(d, 5, y_pos, ">");
                draw_page_str(d, 14, y_pos, items[i]);
            }
            u8g2_SetDrawColor(d, 1); // Reset for next items
        }
        else
        {
            if (bold)
            {
                draw_bold_str(d, 14, y_pos, items[i]);
            }
            else
            {
                draw_page_str(d, 14, y_pos, items[i]);
            }
        }
        y_pos += row_step;
    }

    // 3. Footer
    if (show_footer)
    {
        u8g2_DrawHLine(d, 0, 51, 128);
        u8g2_SetFont(d, u8g2_font_5x7_tf);
        const char *select_text = "E SELECT";
        int select_x = 126 - u8g2_GetStrWidth(d, select_text);
        if (bold)
        {
            draw_bold_str(d, 2, 61, "B BACK");
            draw_bold_str(d, select_x - 1, 61, select_text);
        }
        else
        {
            u8g2_DrawStr(d, 2, 61, "B BACK");
            u8g2_DrawStr(d, select_x, 61, select_text);
        }
    }

    lcd_submit_frame(d);
}

static void draw_menu_list_page_impl(u8g2_t *d, const char *title,
                         const char *const *items, uint8_t item_count,
                         uint8_t selected_index)
{
    draw_menu_list_page_ex(d, title, NULL, items, item_count,
                           selected_index, false, true);
}

static void draw_menu_page_impl(u8g2_t *d, uint8_t selected_index)
{
    const char *menu_items[] = {"WIFI CONFIG", "SENSOR SETTINGS",
                                "PRODUCTION", "SCRAP", "STATE DETECT",
                                "LTE MODE", "USER RESTART"};
    const uint8_t item_count =
        (uint8_t)(sizeof(menu_items) / sizeof(menu_items[0]));
    if (selected_index >= item_count)
    {
        selected_index = 0;
    }
    char page_text[8];
    snprintf(page_text, sizeof(page_text), "%u/%u",
             (unsigned)(selected_index + 1), (unsigned)item_count);
    uint8_t first_visible = (selected_index >= 3) ? selected_index - 2 : 0;
    if (first_visible + 3 > item_count)
    {
        first_visible = item_count - 3;
    }
    const char *visible_items[] = {
        menu_items[first_visible],
        menu_items[first_visible + 1],
        menu_items[first_visible + 2],
    };
    draw_menu_list_page_ex(d, "MENU", page_text, visible_items, 3,
                           selected_index - first_visible, false, true);
}

void lcd_init(u8g2_t *u8g2)
{
    u8g2_Setup_st7920_s_128x64_f(u8g2, LCD_ROTATION, u8x8_byte_4wire_sw_spi, u8g2_esp32_gpio_and_delay_cb);
    u8g2_InitDisplay(u8g2);
    u8g2_SetPowerSave(u8g2, 0);
    s_lcd_draw_mutex = xSemaphoreCreateRecursiveMutex();
    s_lcd_frames = xQueueCreate(1, sizeof(lcd_frame_t));
    if (!s_lcd_draw_mutex || !s_lcd_frames ||
        xTaskCreatePinnedToCore(lcd_transfer_worker, "lcd_tx", 4096, NULL, 2, NULL, 0) != pdPASS) {
        ESP_LOGE("lcd", "LCD worker initialization failed");
        if (s_lcd_frames) vQueueDelete(s_lcd_frames);
        if (s_lcd_draw_mutex) vSemaphoreDelete(s_lcd_draw_mutex);
        s_lcd_frames = NULL;
        s_lcd_draw_mutex = NULL;
    }
}

void lcd_deinit(u8g2_t *u8g2)
{
    // u8g2 doesn't require explicit deinit, just disable power save
    if (u8g2 && s_lcd_draw_mutex &&
        xSemaphoreTakeRecursive(s_lcd_draw_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        s_lcd_staging.context = *u8g2;
        s_lcd_staging.power_save = true;
        xQueueOverwrite(s_lcd_frames, &s_lcd_staging);
        xSemaphoreGiveRecursive(s_lcd_draw_mutex);
    }
}

// ============================================================
// UPDATE HOME DISPLAY WITH SENSOR DATA
// ============================================================

static void lcd_update_home_display_impl(u8g2_t *u8g2, const char *time_str,
                             const char *device_id,
                             machine_state_t state, float current_a,
                             float current_b, float current_c,
                             uint8_t ct_open_mask, float power_value,
                             bool apparent_units,
                             float vibration_mm_s, int temperature_c,
                             const char *network_label, uint8_t signal_level,
                             bool mqtt_connected,
                             const char *reason_code,
                             bool reason_visible, bool reason_ok,
                             bool reason_prompt,
                             const char *status_text,
                             const char *production_value)
{
    if (!u8g2)
        return;

    datameter_home_t home_data = {0};

    // Fill home data structure with provided values
    if (time_str)
    {
        strncpy(home_data.time_text, time_str, sizeof(home_data.time_text) - 1);
    }
    else
    {
        strcpy(home_data.time_text, "00:00");
    }
    snprintf(home_data.device_id, sizeof(home_data.device_id), "%s",
             device_id ? device_id : "");

    home_data.state = state;
    home_data.current_a = current_a;
    home_data.current_b = current_b;
    home_data.current_c = current_c;
    home_data.ct_open_mask = ct_open_mask;
    home_data.power_value = power_value;
    home_data.apparent_units = apparent_units;
    home_data.vibration_mm_s = vibration_mm_s;
    home_data.temperature_c = temperature_c;
    home_data.pressure_bar = 0.0f; // Not used in updated display
    home_data.signal_level = signal_level;
    snprintf(home_data.network_label, sizeof(home_data.network_label), "%s",
             network_label ? network_label : "");
    home_data.mqtt_connected = mqtt_connected;
    home_data.reason_visible = reason_visible;
    home_data.reason_ok = reason_ok;
    home_data.reason_prompt = reason_prompt;
    snprintf(home_data.reason_code, sizeof(home_data.reason_code), "%s",
             reason_code ? reason_code : "__");
    snprintf(home_data.status_text, sizeof(home_data.status_text), "%s",
             status_text ? status_text : "");
    snprintf(home_data.production_value, sizeof(home_data.production_value),
             "%s", production_value ? production_value : "DISABLED");

    // Draw the home page
    draw_home_page(u8g2, &home_data);
}

// ============================================================
// CALIBRATION SETTINGS PAGE
// ============================================================

static void draw_calibration_settings_page_impl(u8g2_t *d, const datameter_calibration_t *cal,
                                    const char *title, uint8_t selected_option)
{
    if (!d || !cal)
        return;

    u8g2_ClearBuffer(d);
    u8g2_SetFontMode(d, 1);

    // 1. Header
    u8g2_SetFont(d, u8g2_font_6x12_tf);
    u8g2_DrawStr(d, 2, 10, title ? title : "CALIBRATION");
    u8g2_DrawHLine(d, 0, 12, 128);

    // 2. Display Axis and Parameter selection
    u8g2_SetFont(d, u8g2_font_5x7_tf);

    // Calibration options layout:
    // Line 1: X-axis Factor
    // Line 2: X-axis Offset
    // Line 3: Y-axis Factor
    // Line 4: Y-axis Offset
    // (Note: You can add Z in a separate page or scroll)

    const char *options[] = {
        "X FACTOR (A)", "X OFFSET (B)",
        "Y FACTOR (A)", "Y OFFSET (B)",
        "Z FACTOR (A)", "Z OFFSET (B)"};

    int y_positions[] = {25, 37, 49, 61};

    // Show up to 4 options at a time
    u8g2_SetFont(d, u8g2_font_5x7_tf);

    for (int i = 0; i < 4; i++)
    {
        int opt_idx = (selected_option / 2) * 2 + i; // Show 2 options per screen
        if (opt_idx >= 6)
            break; // Only 6 calibration options

        int y_pos = y_positions[i];

        if (opt_idx == selected_option)
        {
            // Highlight selected option
            u8g2_SetDrawColor(d, 1);
            u8g2_DrawBox(d, 2, y_pos - 8, 124, 10);
            u8g2_SetDrawColor(d, 0); // Inverse text
            u8g2_DrawStr(d, 5, y_pos, ">");
            u8g2_DrawStr(d, 16, y_pos, options[opt_idx]);
            u8g2_SetDrawColor(d, 1); // Reset
        }
        else
        {
            u8g2_DrawStr(d, 16, y_pos, options[opt_idx]);
        }
    }

    // 3. Footer with instructions
    u8g2_DrawHLine(d, 0, 54, 128);
    u8g2_SetFont(d, u8g2_font_6x12_tf);
    u8g2_DrawStr(d, 5, 63, "UP/DN");
    u8g2_DrawStr(d, 55, 63, "SELECT");

    // Show password requirement for Factor
    if (selected_option % 2 == 0)
    { // Even indices are Factor
        u8g2_SetFont(d, u8g2_font_4x6_tf);
        u8g2_DrawStr(d, 110, 20, "PWD*");
    }

    lcd_submit_frame(d);
}

void draw_startup_logo_animation(u8g2_t *d) {
    if (!s_lcd_draw_mutex || xSemaphoreTakeRecursive(s_lcd_draw_mutex, pdMS_TO_TICKS(10)) != pdTRUE) {
        return;
    }
    draw_startup_logo_animation_impl(d);
    xSemaphoreGiveRecursive(s_lcd_draw_mutex);
}

void draw_boot_identity_page(u8g2_t *d, const char *device_id,
                             const char *boot_reason) {
    if (!s_lcd_draw_mutex || xSemaphoreTakeRecursive(s_lcd_draw_mutex, pdMS_TO_TICKS(10)) != pdTRUE) {
        return;
    }
    draw_boot_identity_page_impl(d, device_id, boot_reason);
    xSemaphoreGiveRecursive(s_lcd_draw_mutex);
}

void draw_home_page(u8g2_t *d, const datameter_home_t *m) {
    if (!s_lcd_draw_mutex || xSemaphoreTakeRecursive(s_lcd_draw_mutex, pdMS_TO_TICKS(10)) != pdTRUE) {
        return;
    }
    draw_home_page_impl(d, m);
    xSemaphoreGiveRecursive(s_lcd_draw_mutex);
}

void draw_energy_page(u8g2_t *d, const datameter_energy_t *e) {
    if (!s_lcd_draw_mutex || xSemaphoreTakeRecursive(s_lcd_draw_mutex, pdMS_TO_TICKS(10)) != pdTRUE) {
        return;
    }
    draw_energy_page_impl(d, e);
    xSemaphoreGiveRecursive(s_lcd_draw_mutex);
}

void draw_speed_page(u8g2_t *d, const datameter_speed_t *s) {
    if (!s_lcd_draw_mutex || xSemaphoreTakeRecursive(s_lcd_draw_mutex, pdMS_TO_TICKS(10)) != pdTRUE) {
        return;
    }
    draw_speed_page_impl(d, s);
    xSemaphoreGiveRecursive(s_lcd_draw_mutex);
}

void draw_duration_page(u8g2_t *d, const datameter_duration_t *p) {
    if (!s_lcd_draw_mutex || xSemaphoreTakeRecursive(s_lcd_draw_mutex, pdMS_TO_TICKS(10)) != pdTRUE) {
        return;
    }
    draw_duration_page_impl(d, p);
    xSemaphoreGiveRecursive(s_lcd_draw_mutex);
}

void draw_count_page(u8g2_t *d, const datameter_count_t *c) {
    if (!s_lcd_draw_mutex || xSemaphoreTakeRecursive(s_lcd_draw_mutex, pdMS_TO_TICKS(10)) != pdTRUE) {
        return;
    }
    draw_count_page_impl(d, c);
    xSemaphoreGiveRecursive(s_lcd_draw_mutex);
}

void draw_vibration_page(u8g2_t *d, const datameter_vibration_t *v) {
    if (!s_lcd_draw_mutex || xSemaphoreTakeRecursive(s_lcd_draw_mutex, pdMS_TO_TICKS(10)) != pdTRUE) {
        return;
    }
    draw_vibration_page_impl(d, v);
    xSemaphoreGiveRecursive(s_lcd_draw_mutex);
}

void draw_phase_page(u8g2_t *d, const datameter_phase_t *p) {
    if (!s_lcd_draw_mutex || xSemaphoreTakeRecursive(s_lcd_draw_mutex, pdMS_TO_TICKS(10)) != pdTRUE) {
        return;
    }
    draw_phase_page_impl(d, p);
    xSemaphoreGiveRecursive(s_lcd_draw_mutex);
}

void draw_encoder_page(u8g2_t *d, const datameter_encoder_t *e) {
    if (!s_lcd_draw_mutex || xSemaphoreTakeRecursive(s_lcd_draw_mutex, pdMS_TO_TICKS(10)) != pdTRUE) {
        return;
    }
    draw_encoder_page_impl(d, e);
    xSemaphoreGiveRecursive(s_lcd_draw_mutex);
}

void draw_encoder_count_page(u8g2_t *d, const datameter_encoder_count_t *e) {
    if (!s_lcd_draw_mutex || xSemaphoreTakeRecursive(s_lcd_draw_mutex, pdMS_TO_TICKS(10)) != pdTRUE) {
        return;
    }
    draw_encoder_count_page_impl(d, e);
    xSemaphoreGiveRecursive(s_lcd_draw_mutex);
}

void draw_proxy_page(u8g2_t *d, const datameter_proxy_t *p) {
    if (!s_lcd_draw_mutex || xSemaphoreTakeRecursive(s_lcd_draw_mutex, pdMS_TO_TICKS(10)) != pdTRUE) {
        return;
    }
    draw_proxy_page_impl(d, p);
    xSemaphoreGiveRecursive(s_lcd_draw_mutex);
}

void draw_proxy_production_page(u8g2_t *d,
                                const datameter_proxy_production_t *p) {
    if (!s_lcd_draw_mutex || xSemaphoreTakeRecursive(s_lcd_draw_mutex, pdMS_TO_TICKS(10)) != pdTRUE) {
        return;
    }
    draw_proxy_production_page_impl(d, p);
    xSemaphoreGiveRecursive(s_lcd_draw_mutex);
}

void draw_lte_diag_page(u8g2_t *d, const datameter_lte_diag_t *lte) {
    if (!s_lcd_draw_mutex || xSemaphoreTakeRecursive(s_lcd_draw_mutex, pdMS_TO_TICKS(10)) != pdTRUE) {
        return;
    }
    draw_lte_diag_page_impl(d, lte);
    xSemaphoreGiveRecursive(s_lcd_draw_mutex);
}

void draw_wifi_diag_page(u8g2_t *d, const datameter_wifi_diag_t *wifi) {
    if (!s_lcd_draw_mutex || xSemaphoreTakeRecursive(s_lcd_draw_mutex, pdMS_TO_TICKS(10)) != pdTRUE) {
        return;
    }
    draw_wifi_diag_page_impl(d, wifi);
    xSemaphoreGiveRecursive(s_lcd_draw_mutex);
}

void draw_text_page(u8g2_t *d, const char *title, const char *line0,
                    const char *line1, const char *line2,
                    const char *line3) {
    if (!s_lcd_draw_mutex || xSemaphoreTakeRecursive(s_lcd_draw_mutex, pdMS_TO_TICKS(10)) != pdTRUE) {
        return;
    }
    draw_text_page_impl(d, title, line0, line1, line2, line3);
    xSemaphoreGiveRecursive(s_lcd_draw_mutex);
}

void draw_text_page_normal(u8g2_t *d, const char *title, const char *line0,
                           const char *line1, const char *line2,
                           const char *line3) {
    if (!s_lcd_draw_mutex || xSemaphoreTakeRecursive(s_lcd_draw_mutex, pdMS_TO_TICKS(10)) != pdTRUE) {
        return;
    }
    draw_text_page_normal_impl(d, title, line0, line1, line2, line3);
    xSemaphoreGiveRecursive(s_lcd_draw_mutex);
}

void draw_reason_search_page(u8g2_t *d, const char *search_text,
                             const char *const *items, uint8_t item_count,
                             const char *status_text) {
    if (!s_lcd_draw_mutex || xSemaphoreTakeRecursive(s_lcd_draw_mutex, pdMS_TO_TICKS(10)) != pdTRUE) {
        return;
    }
    draw_reason_search_page_impl(d, search_text, items, item_count, status_text);
    xSemaphoreGiveRecursive(s_lcd_draw_mutex);
}

void draw_menu_list_page_ex(u8g2_t *d, const char *title,
                            const char *page_text,
                            const char *const *items, uint8_t item_count,
                            uint8_t selected_index, bool bold,
                            bool show_footer) {
    if (!s_lcd_draw_mutex || xSemaphoreTakeRecursive(s_lcd_draw_mutex, pdMS_TO_TICKS(10)) != pdTRUE) {
        return;
    }
    draw_menu_list_page_ex_impl(d, title, page_text, items, item_count, selected_index, bold, show_footer);
    xSemaphoreGiveRecursive(s_lcd_draw_mutex);
}

void draw_menu_list_page(u8g2_t *d, const char *title,
                         const char *const *items, uint8_t item_count,
                         uint8_t selected_index) {
    if (!s_lcd_draw_mutex || xSemaphoreTakeRecursive(s_lcd_draw_mutex, pdMS_TO_TICKS(10)) != pdTRUE) {
        return;
    }
    draw_menu_list_page_impl(d, title, items, item_count, selected_index);
    xSemaphoreGiveRecursive(s_lcd_draw_mutex);
}

void draw_menu_page(u8g2_t *d, uint8_t selected_index) {
    if (!s_lcd_draw_mutex || xSemaphoreTakeRecursive(s_lcd_draw_mutex, pdMS_TO_TICKS(10)) != pdTRUE) {
        return;
    }
    draw_menu_page_impl(d, selected_index);
    xSemaphoreGiveRecursive(s_lcd_draw_mutex);
}

void lcd_update_home_display(u8g2_t *u8g2, const char *time_str,
                             const char *device_id,
                             machine_state_t state, float current_a,
                             float current_b, float current_c,
                             uint8_t ct_open_mask, float power_value,
                             bool apparent_units,
                             float vibration_mm_s, int temperature_c,
                             const char *network_label, uint8_t signal_level,
                             bool mqtt_connected,
                             const char *reason_code,
                             bool reason_visible, bool reason_ok,
                             bool reason_prompt,
                             const char *status_text,
                             const char *production_value) {
    if (!s_lcd_draw_mutex || xSemaphoreTakeRecursive(s_lcd_draw_mutex, pdMS_TO_TICKS(10)) != pdTRUE) {
        return;
    }
    lcd_update_home_display_impl(u8g2, time_str, device_id, state, current_a, current_b, current_c, ct_open_mask, power_value, apparent_units, vibration_mm_s, temperature_c, network_label, signal_level, mqtt_connected, reason_code, reason_visible, reason_ok, reason_prompt, status_text, production_value);
    xSemaphoreGiveRecursive(s_lcd_draw_mutex);
}

void draw_calibration_settings_page(u8g2_t *d, const datameter_calibration_t *cal,
                                    const char *title, uint8_t selected_option) {
    if (!s_lcd_draw_mutex || xSemaphoreTakeRecursive(s_lcd_draw_mutex, pdMS_TO_TICKS(10)) != pdTRUE) {
        return;
    }
    draw_calibration_settings_page_impl(d, cal, title, selected_option);
    xSemaphoreGiveRecursive(s_lcd_draw_mutex);
}
