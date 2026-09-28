#include "app_config.h"

#include "cavli_lte_mgr.h"
#include "datameter_lcd.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "u8g2.h"

#include <limits.h>
#include <stdio.h>
#include <string.h>

static const char *TAG = "lte_lcd_app";
static u8g2_t s_lcd;

static void copy_or_placeholder(char *dst, size_t dst_size,
                                const char *value, const char *fallback)
{
    snprintf(dst, dst_size, "%s", (value && value[0]) ? value : fallback);
}

static void draw_lte_page(void)
{
    cavli_diag_t diag;
    datameter_lte_diag_t page = {0};

    cavli_lte_mgr_get_diag(&diag);
    page.meta.page_index = 0;
    page.meta.page_count = 2;
    page.meta.has_prev = true;
    page.meta.has_next = true;

    copy_or_placeholder(page.network_text, sizeof(page.network_text),
                        diag.operator_name, "NO NET");
    copy_or_placeholder(page.sim_text, sizeof(page.sim_text),
                        diag.sim_ready ? "READY" : "NOT RDY", "NOT RDY");
    copy_or_placeholder(page.band_text, sizeof(page.band_text), diag.band, "--");
    copy_or_placeholder(page.ip_text, sizeof(page.ip_text), diag.ip_addr, "--");
    snprintf(page.mqtt_text, sizeof(page.mqtt_text), "MQTT:%s",
             diag.connected ? "CONNECTED" : "DISCONNECTED");
    page.rssi_dbm = diag.rssi_dbm;
    page.meta.status_text = cavli_state_to_str(diag.state);

    draw_lte_diag_page(&s_lcd, &page);
}

static void draw_mqtt_page(void)
{
    cavli_diag_t diag;
    char line0[32], line1[32], line2[32], line3[32];

    cavli_lte_mgr_get_diag(&diag);
    snprintf(line0, sizeof(line0), "STATUS: %s",
             diag.connected ? "CONNECTED" : "OFFLINE");
    snprintf(line1, sizeof(line1), "SENT:%lu FAIL:%lu",
             (unsigned long)diag.messages_sent,
             (unsigned long)diag.messages_failed);
    snprintf(line2, sizeof(line2), "CODE:%d RECON:%lu",
             (int)diag.mqtt_reason_code,
             (unsigned long)diag.mqtt_reconnects);
    snprintf(line3, sizeof(line3), "ERR:%s",
             cavli_error_to_str(diag.last_error));

    draw_text_page_normal(&s_lcd, "-- MQTT DIAG --", line0, line1, line2,
                          line3);
}

static void display_task(void *arg)
{
    (void)arg;
    bool mqtt_page = false;

    while (true) {
        (void)cavli_lte_mgr_refresh_diag_async();
        if (mqtt_page) {
            draw_mqtt_page();
        } else {
            draw_lte_page();
        }
        mqtt_page = !mqtt_page;
        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Starting standalone LTE + LCD diagnostics");

    lcd_init(&s_lcd);

    /* Submit a frame immediately.  Modem startup can take several seconds;
     * the LCD must show a known-good boot screen independently of LTE state. */
    draw_text_page_normal(&s_lcd, "LTE LCD", "LCD ONLINE",
                          "Starting modem...", "Please wait", "BOOT");

    /* Start the UI before modem startup.  LTE registration can take up to a
     * minute, but the LCD must show the live NO NET/registration state during
     * that time instead of remaining on the boot frame. */
    if (xTaskCreate(display_task, "lte_lcd_display", 4096, NULL, 2, NULL) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create LCD diagnostics task");
    }

    esp_err_t err = cavli_lte_mgr_init(
        MQTT_BROKER_HOST, MQTT_BROKER_PORT, MQTT_CLIENT_ID,
        MQTT_USERNAME, MQTT_PASSWORD, MQTT_BASE_TOPIC, MQTT_KEEPALIVE_SEC);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "LTE manager init failed: %s", esp_err_to_name(err));
    } else {
        err = cavli_lte_mgr_start();
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "LTE manager start failed: %s", esp_err_to_name(err));
        }
    }

}
