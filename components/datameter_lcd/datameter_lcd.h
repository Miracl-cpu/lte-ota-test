#ifndef DATAMETER_LCD_H
#define DATAMETER_LCD_H

#include <stdint.h>
#include <stdbool.h>
#include "u8g2.h"

#define LCD_ROTATION U8G2_R2

typedef enum {
    MACHINE_RUN = 0,
    MACHINE_IDLE,
    MACHINE_STOP,
    MACHINE_OFF
} machine_state_t;

typedef struct {
    char time_text[12];
    char device_id[16];
    machine_state_t state;
    float current_a;
    float current_b;
    float current_c;
    uint8_t ct_open_mask;
    float power_value;
    bool apparent_units;
    float vibration_mm_s;
    int temperature_c;
    float pressure_bar;
    uint8_t signal_level;
    char network_label[4];
    bool mqtt_connected;
    bool reason_visible;
    bool reason_ok;
    bool reason_prompt;
    char reason_code[4];
    char status_text[18];
    char production_value[16];
} datameter_home_t;

typedef struct {
    uint8_t page_index;
    uint8_t page_count;
    bool has_prev;
    bool has_next;
    const char *status_text;
} datameter_page_meta_t;

typedef struct {
    datameter_page_meta_t meta;
    float active_kwh;
    float standby_kwh;
    float today_kwh;
    float total_kwh;
    bool apparent_units;
} datameter_energy_t;

typedef struct {
    datameter_page_meta_t meta;
    float current_length_m;
    float total_length_m;
    float speed_mps;
} datameter_speed_t;

typedef struct {
    datameter_page_meta_t meta;
    char title[16];
    bool selected_valid;
    uint32_t count;
    uint32_t part_number;
    uint16_t cycle_time_s;
    uint16_t operations_per_part;
    uint16_t operation_count;
} datameter_duration_t;

typedef struct {
    datameter_page_meta_t meta;
    uint32_t count;
    float current_a;
    float threshold_a;
    bool enabled;
} datameter_count_t;

typedef struct {
    datameter_page_meta_t meta;
    float displacement_um;
    float acceleration_g;
    float velocity_mm_s;
    float frequency_hz;
} datameter_vibration_t;

typedef struct {
    datameter_page_meta_t meta;
    float current_a;
    float current_b;
    float current_c;
    uint8_t ct_open_mask;
    bool single_phase;
} datameter_phase_t;

typedef struct {
    datameter_page_meta_t meta;
    float rpm;
    float mpm;
    float length_m;
} datameter_encoder_t;

typedef struct {
    datameter_page_meta_t meta;
    float current_length_m;
    uint32_t good_count;
    uint32_t scrap_count;
} datameter_encoder_count_t;

typedef struct {
    datameter_page_meta_t meta;
    uint32_t count_a;
    uint32_t count_b;
    uint32_t count_total;
} datameter_proxy_t;

typedef struct {
    datameter_page_meta_t meta;
    char title[16];
    bool meter_mode;
    bool scrap_enabled;
    float total_length_m;
    uint32_t scrap_count;
    uint32_t good_count;
} datameter_proxy_production_t;

typedef struct {
    datameter_page_meta_t meta;
    char network_text[32];
    char sim_text[12];
    char band_text[16];
    char apn_text[32];
    char ip_text[32];
    char mqtt_text[40];
    int16_t rsrp_dbm;
    int16_t rsrq_db;
    int16_t rssi_dbm;
} datameter_lte_diag_t;

typedef struct {
    datameter_page_meta_t meta;
    char ssid_text[32];
    char ip_text[16];
    char wifi_text[16];
    char mqtt_text[16];
    char mode_text[16];
    int8_t rssi_dbm;
} datameter_wifi_diag_t;

void lcd_init(u8g2_t *u8g2);
void lcd_deinit(u8g2_t *u8g2);

// Helper function to update display with sensor data
void lcd_update_home_display(u8g2_t *u8g2, const char *time_str,
                             const char *device_id,
                             machine_state_t state, float current_a,
                             float current_b, float current_c,
                             uint8_t ct_open_mask,
                             float power_value, bool apparent_units,
                             float vibration_mm_s, int temperature_c,
                             const char *network_label,
                             uint8_t signal_level,
                             bool mqtt_connected,
                             const char *reason_code, bool reason_visible,
                             bool reason_ok, bool reason_prompt,
                             const char *status_text,
                             const char *production_value);

void draw_home_page(u8g2_t *d, const datameter_home_t *m);
void draw_startup_logo_animation(u8g2_t *d);
void draw_boot_identity_page(u8g2_t *d, const char *device_id,
                             const char *boot_reason);
void draw_energy_page(u8g2_t *d, const datameter_energy_t *e);
void draw_speed_page(u8g2_t *d, const datameter_speed_t *s);
void draw_duration_page(u8g2_t *d, const datameter_duration_t *p);
void draw_count_page(u8g2_t *d, const datameter_count_t *c);
void draw_vibration_page(u8g2_t *d, const datameter_vibration_t *v);
void draw_phase_page(u8g2_t *d, const datameter_phase_t *p);
void draw_encoder_page(u8g2_t *d, const datameter_encoder_t *e);
void draw_encoder_count_page(u8g2_t *d, const datameter_encoder_count_t *e);
void draw_proxy_page(u8g2_t *d, const datameter_proxy_t *p);
void draw_proxy_production_page(u8g2_t *d,
                                const datameter_proxy_production_t *p);
void draw_lte_diag_page(u8g2_t *d, const datameter_lte_diag_t *lte);
void draw_wifi_diag_page(u8g2_t *d, const datameter_wifi_diag_t *wifi);
void draw_text_page(u8g2_t *d, const char *title, const char *line0,
                    const char *line1, const char *line2,
                    const char *line3);
void draw_text_page_normal(u8g2_t *d, const char *title, const char *line0,
                           const char *line1, const char *line2,
                           const char *line3);
void draw_reason_search_page(u8g2_t *d, const char *search_text,
                             const char *const *items, uint8_t item_count,
                             const char *status_text);
void draw_menu_page(u8g2_t *d, uint8_t selected_index);
void draw_menu_list_page(u8g2_t *d, const char *title,
                         const char *const *items, uint8_t item_count,
                         uint8_t selected_index);
void draw_menu_list_page_ex(u8g2_t *d, const char *title,
                            const char *page_text,
                            const char *const *items, uint8_t item_count,
                            uint8_t selected_index, bool bold,
                            bool show_footer);

// Calibration structure and functions
typedef struct {
    uint16_t factor;      // Speed Coefficient A (requires password to change)
    uint16_t offset;      // Speed Coefficient B (no password required)
    const char *status;   // Status message ("OK", "Writing...", "Error", etc)
} datameter_calibration_t;

void draw_calibration_settings_page(u8g2_t *d, const datameter_calibration_t *cal,
                                     const char *title, uint8_t selected_option);

void lcd_get_runtime_stats(uint32_t *completed, uint32_t *slow,
                           uint32_t *recoveries, uint32_t *last_ms);

#endif
