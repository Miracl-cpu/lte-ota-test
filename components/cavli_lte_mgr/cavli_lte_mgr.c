/**
 * @file cavli_lte_mgr.c
 * @brief Cavli C16qs LTE module manager — implementation
 *
 * Block 1: UART2 init + at_send_wait() + basic AT smoke test.
 * Block 2: cavli_connect_once() — full network registration (steps 0-11).
 * Subsequent blocks will add MQTT connect/publish, UART RX task, reconnect task.
 */

#include "log_config.h"
#undef LOG_LOCAL_LEVEL
#define LOG_LOCAL_LEVEL LOG_LEVEL_CAVLI
#include "esp_log.h"
#include "esp_timer.h"

#include <limits.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>
#include "cJSON.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/uart.h"
#include "esp_task_wdt.h"

#include "cavli_lte_mgr.h"
#include "json_data_logger.h"

static const char *TAG = "cavli_lte_mgr";

/* ============================================================================
 * Internal state
 * ========================================================================= */

static int               g_mqtt_token    = -1;   // Token from AT+MQTTCREATE; -1 = none
static cavli_diag_t      g_diag          = {0};
static SemaphoreHandle_t g_diag_mutex    = NULL;  // Protects g_diag (RX task + reconnect task)
static SemaphoreHandle_t g_uart_mutex    = NULL;  // Serialises all UART2 AT send+read cycles

static char     g_base_topic[64];
static char     g_lwt_topic[96];         // <base_topic>/lwt
static char     g_config_topic[96];      // <base_topic>/config
static char     g_config_ack_topic[96];  // <base_topic>/config/ack

static char     g_broker[64];
static uint16_t g_port;
static char     g_client_id[32];
static char     g_username[32];
static char     g_password[64];
static uint16_t g_keepalive;

static volatile bool g_stop_requested      = false; // Set by cavli_lte_mgr_stop()
static cavli_config_cb_t g_config_cb        = NULL;  // Registered via cavli_lte_mgr_set_config_callback()
static TaskHandle_t  g_rx_task_handle       = NULL;  // UART RX task handle — set NULL by task on exit
static TaskHandle_t  g_reconnect_task_handle = NULL; // Reconnect task handle — set NULL by task on exit
static TaskHandle_t  g_diag_refresh_task_handle = NULL;
static bool          g_uart_driver_installed = false;

/* Reconnect backoff constants */
#define CAVLI_RECONNECT_BACKOFF_INITIAL_MS  60000U   // 60 s first retry after failure
#define CAVLI_RECONNECT_BACKOFF_MAX_MS     300000U   // 300 s cap
#define CAVLI_PUBLISH_FAILURE_LIMIT              3U
#define CAVLI_FULL_FAILURES_BEFORE_RESET          3U
#define CAVLI_MODEM_RESET_COOLDOWN_MS        900000U  // 15 minutes

/* ============================================================================
 * Internal helpers — diagnostic
 * ========================================================================= */

/* Update g_diag.state under mutex — reduces boilerplate in cavli_connect_once() */
#define UPDATE_STATE(s) do {                         \
    xSemaphoreTake(g_diag_mutex, portMAX_DELAY);     \
    g_diag.state = (s);                              \
    xSemaphoreGive(g_diag_mutex);                    \
} while (0)

/* Set last_error under mutex and log the full diagnostic — call on any abort */
#define SET_ABORT(e) do {                            \
    xSemaphoreTake(g_diag_mutex, portMAX_DELAY);     \
    g_diag.last_error = (e);                         \
    xSemaphoreGive(g_diag_mutex);                    \
    cavli_log_diag_abort();                          \
} while (0)

static void cavli_log_diag_abort(void)
{
    xSemaphoreTake(g_diag_mutex, portMAX_DELAY);
    cavli_state_t s = g_diag.state;
    cavli_error_t e = g_diag.last_error;
    char detail[64];
    strlcpy(detail, g_diag.error_detail, sizeof(detail));
    int8_t rssi = g_diag.signal_rssi;
    xSemaphoreGive(g_diag_mutex);

    ESP_LOGE(TAG, "[CAVLI] FAILED at stage : %s", cavli_state_to_str(s));
    ESP_LOGE(TAG, "[CAVLI] Error           : %s", cavli_error_to_str(e));
    if (detail[0]) ESP_LOGE(TAG, "[CAVLI] CME detail      : %s", detail);
    if (rssi != 0) ESP_LOGE(TAG, "[CAVLI] Signal RSSI     : %d", rssi);
}

static esp_err_t at_send_wait_ex(const char *cmd,
                                 const char *expected1,
                                 const char *expected2,
                                 uint32_t timeout_ms,
                                 char *out_buf, size_t out_size);

static void cavli_diag_set_defaults(cavli_diag_t *diag)
{
    if (!diag) {
        return;
    }
    diag->signal_rssi = 99;
    diag->rssi_dbm = INT16_MIN;
    diag->rsrp_dbm = INT16_MIN;
    diag->rsrq_db = INT16_MIN;
    diag->mqtt_reason_code = 0;
    snprintf(diag->last_recovery_action,
             sizeof(diag->last_recovery_action), "NONE");
}

static void cavli_wake_reconnect_task(void)
{
    TaskHandle_t task = g_reconnect_task_handle;
    if (task != NULL) {
        xTaskNotifyGive(task);
    }
}

static int16_t cavli_csq_to_dbm(int csq)
{
    if (csq < 0 || csq > 31 || csq == 99) {
        return INT16_MIN;
    }
    return (int16_t)(-113 + (2 * csq));
}

static int64_t cavli_days_from_civil(int year, int month, int day)
{
    year -= (month <= 2);
    const int era = (year >= 0 ? year : year - 399) / 400;
    const unsigned yoe = (unsigned)(year - era * 400);
    const unsigned mp = (unsigned)(month + (month > 2 ? -3 : 9));
    const unsigned doy = (153U * mp + 2U) / 5U + (unsigned)day - 1U;
    const unsigned doe = yoe * 365U + yoe / 4U - yoe / 100U + doy;
    return (int64_t)era * 146097LL + (int64_t)doe - 719468LL;
}

static bool cavli_parse_cclk_time(const char *resp, struct timeval *out_tv)
{
    if (!resp || !out_tv) {
        return false;
    }

    const char *quote = strchr(resp, '"');
    if (!quote) {
        return false;
    }

    int yy = 0, month = 0, day = 0, hour = 0, min = 0, sec = 0, tz_quarters = 0;
    char tz_sign = '+';
    if (sscanf(quote + 1, "%2d/%2d/%2d,%2d:%2d:%2d%c%2d",
               &yy, &month, &day, &hour, &min, &sec, &tz_sign, &tz_quarters) != 8) {
        return false;
    }

    int year = (yy >= 70) ? (1900 + yy) : (2000 + yy);
    if (year < 2020 || month < 1 || month > 12 || day < 1 || day > 31 ||
        hour < 0 || hour > 23 || min < 0 || min > 59 || sec < 0 || sec > 60 ||
        (tz_sign != '+' && tz_sign != '-') || tz_quarters < 0 || tz_quarters > 96) {
        return false;
    }

    int tz_offset_sec = tz_quarters * 15 * 60;
    if (tz_sign == '-') {
        tz_offset_sec = -tz_offset_sec;
    }

    int64_t epoch = cavli_days_from_civil(year, month, day) * 86400LL +
                    (int64_t)hour * 3600LL + (int64_t)min * 60LL + sec -
                    tz_offset_sec;
    if (epoch < 1577836800LL) { // 2020-01-01T00:00:00Z
        return false;
    }

    out_tv->tv_sec = (time_t)epoch;
    out_tv->tv_usec = 0;
    return true;
}

static esp_err_t cavli_sync_system_time_from_modem(void)
{
    // These are harmless if unsupported; AT+CCLK? below is the real source.
    at_send_wait_ex("AT+CTZU=1", "OK", "ERROR", 1000, NULL, 0);
    at_send_wait_ex("AT+CLTS=1", "OK", "ERROR", 1000, NULL, 0);

    for (int attempt = 0; attempt < 5; attempt++) {
        char resp[128] = {0};
        esp_err_t ret = at_send_wait_ex("AT+CCLK?", "+CCLK:", "ERROR",
                                        CAVLI_AT_TIMEOUT_MS, resp, sizeof(resp));
        if (ret == ESP_OK) {
            struct timeval tv = {0};
            if (cavli_parse_cclk_time(resp, &tv)) {
                if (settimeofday(&tv, NULL) == 0) {
                    time_t now = tv.tv_sec;
                    struct tm timeinfo;
                    localtime_r(&now, &timeinfo);
                    char ts[32];
                    strftime(ts, sizeof(ts), "%Y-%m-%dT%H:%M:%S", &timeinfo);
                    ESP_LOGI(TAG, "[CAVLI] System time synced from LTE clock: %s", ts);
                    return ESP_OK;
                }
                ESP_LOGW(TAG, "[CAVLI] settimeofday failed for LTE clock");
            } else {
                ESP_LOGW(TAG, "[CAVLI] LTE clock not valid yet: %s", resp);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(2000));
    }

    ESP_LOGW(TAG, "[CAVLI] LTE clock sync unavailable after retries");
    return ESP_ERR_INVALID_RESPONSE;
}

static int16_t cavli_parse_reason_code(const char *line)
{
    if (!line) {
        return 0;
    }
    const char *comma = strrchr(line, ',');
    if (!comma || !comma[1]) {
        return 0;
    }
    char *end = NULL;
    long value = strtol(comma + 1, &end, 10);
    if (end == comma + 1) {
        return 0;
    }
    return (int16_t)value;
}

static void cavli_copy_token(char *dst, size_t dst_len,
                             const char *start, const char *end)
{
    if (!dst || dst_len == 0) {
        return;
    }
    dst[0] = '\0';
    if (!start || !end || end <= start) {
        return;
    }
    size_t len = (size_t)(end - start);
    if (len >= dst_len) {
        len = dst_len - 1;
    }
    memcpy(dst, start, len);
    dst[len] = '\0';
}

static void cavli_parse_operator_name(const char *resp, char *out, size_t out_len)
{
    if (!resp || !out || out_len == 0) {
        return;
    }
    const char *q1 = strchr(resp, '"');
    if (!q1) {
        return;
    }
    const char *q2 = strchr(q1 + 1, '"');
    if (!q2) {
        return;
    }
    cavli_copy_token(out, out_len, q1 + 1, q2);
}

static void cavli_parse_apn(const char *resp, char *out, size_t out_len)
{
    if (!resp || !out || out_len == 0) {
        return;
    }
    const char *ctx = strstr(resp, "+CGDCONT:");
    if (!ctx) {
        return;
    }
    const char *first = strchr(ctx, '"');
    if (!first) {
        return;
    }
    const char *second = strchr(first + 1, '"');
    if (!second) {
        return;
    }
    const char *third = strchr(second + 1, '"');
    if (!third) {
        return;
    }
    const char *fourth = strchr(third + 1, '"');
    if (!fourth) {
        return;
    }
    cavli_copy_token(out, out_len, third + 1, fourth);
}

static void cavli_parse_ip_addr(const char *resp, char *out, size_t out_len)
{
    if (!resp || !out || out_len == 0) {
        return;
    }
    const char *line = strstr(resp, "+CGPADDR:");
    if (!line) {
        return;
    }
    const char *comma = strchr(line, ',');
    if (!comma) {
        return;
    }
    const char *end = strpbrk(comma + 1, "\r\n");
    if (!end) {
        end = comma + 1 + strlen(comma + 1);
    }
    cavli_copy_token(out, out_len, comma + 1, end);
}

static void cavli_parse_band(const char *resp, char *out, size_t out_len)
{
    if (!resp || !out || out_len == 0) {
        return;
    }
    const char *band = strstr(resp, "BAND");
    if (!band) {
        band = strstr(resp, "Band");
    }
    if (!band) {
        const char *b = strstr(resp, ",B");
        if (b) {
            const char *end = strpbrk(b + 1, ",\r\n");
            if (!end) {
                end = b + 1 + strlen(b + 1);
            }
            cavli_copy_token(out, out_len, b + 1, end);
        }
        return;
    }
    const char *end = strpbrk(band, ",\r\n ");
    if (!end) {
        end = band + strlen(band);
    }
    cavli_copy_token(out, out_len, band, end);
}

static void cavli_refresh_diag_once(void)
{
    if (!g_diag_mutex || !g_uart_mutex) {
        return;
    }

    char resp[256];

    memset(resp, 0, sizeof(resp));
    if (at_send_wait_ex("AT+CPIN?", "OK", NULL, CAVLI_AT_TIMEOUT_MS,
                        resp, sizeof(resp)) == ESP_OK) {
        bool sim_ready = strstr(resp, "+CPIN: READY") != NULL;
        xSemaphoreTake(g_diag_mutex, portMAX_DELAY);
        g_diag.sim_ready = sim_ready;
        xSemaphoreGive(g_diag_mutex);
    }

    memset(resp, 0, sizeof(resp));
    if (at_send_wait_ex("AT+COPS?", "OK", NULL, CAVLI_AT_TIMEOUT_MS,
                        resp, sizeof(resp)) == ESP_OK) {
        char operator_name[sizeof(g_diag.operator_name)] = {0};
        cavli_parse_operator_name(resp, operator_name, sizeof(operator_name));
        if (operator_name[0]) {
            xSemaphoreTake(g_diag_mutex, portMAX_DELAY);
            strlcpy(g_diag.operator_name, operator_name,
                    sizeof(g_diag.operator_name));
            xSemaphoreGive(g_diag_mutex);
        }
    }

    memset(resp, 0, sizeof(resp));
    if (at_send_wait_ex("AT+CSQ", "+CSQ:", NULL, CAVLI_AT_TIMEOUT_MS,
                        resp, sizeof(resp)) == ESP_OK) {
        char *p = strstr(resp, "+CSQ:");
        if (p) {
            p += 5;
            while (*p == ' ') {
                p++;
            }
            int csq = (int)strtol(p, NULL, 10);
            xSemaphoreTake(g_diag_mutex, portMAX_DELAY);
            g_diag.signal_rssi = (int8_t)csq;
            g_diag.rssi_dbm = cavli_csq_to_dbm(csq);
            xSemaphoreGive(g_diag_mutex);
        }
    }

    memset(resp, 0, sizeof(resp));
    if (at_send_wait_ex("AT+CESQ", "+CESQ:", "ERROR", CAVLI_AT_TIMEOUT_MS,
                        resp, sizeof(resp)) == ESP_OK) {
        int rxlev = 0, ber = 0, rscp = 0, ecno = 0, rsrq = 255, rsrp = 255;
        char *p = strstr(resp, "+CESQ:");
        if (p && sscanf(p, "+CESQ: %d,%d,%d,%d,%d,%d",
                        &rxlev, &ber, &rscp, &ecno, &rsrq, &rsrp) == 6) {
            xSemaphoreTake(g_diag_mutex, portMAX_DELAY);
            g_diag.rsrq_db = (rsrq == 255) ? INT16_MIN
                                           : (int16_t)lroundf(-19.5f + (0.5f * rsrq));
            g_diag.rsrp_dbm = (rsrp == 255) ? INT16_MIN
                                            : (int16_t)(-140 + rsrp);
            xSemaphoreGive(g_diag_mutex);
        }
    }

    memset(resp, 0, sizeof(resp));
    if (at_send_wait_ex("AT+CGDCONT?", "OK", NULL, CAVLI_AT_TIMEOUT_MS,
                        resp, sizeof(resp)) == ESP_OK) {
        char apn[sizeof(g_diag.apn)] = {0};
        cavli_parse_apn(resp, apn, sizeof(apn));
        if (apn[0]) {
            xSemaphoreTake(g_diag_mutex, portMAX_DELAY);
            strlcpy(g_diag.apn, apn, sizeof(g_diag.apn));
            xSemaphoreGive(g_diag_mutex);
        }
    }

    memset(resp, 0, sizeof(resp));
    if (at_send_wait_ex("AT+CGPADDR=1", "OK", NULL, CAVLI_AT_TIMEOUT_MS,
                        resp, sizeof(resp)) == ESP_OK) {
        char ip_addr[sizeof(g_diag.ip_addr)] = {0};
        cavli_parse_ip_addr(resp, ip_addr, sizeof(ip_addr));
        if (ip_addr[0]) {
            xSemaphoreTake(g_diag_mutex, portMAX_DELAY);
            strlcpy(g_diag.ip_addr, ip_addr, sizeof(g_diag.ip_addr));
            xSemaphoreGive(g_diag_mutex);
        }
    }

    memset(resp, 0, sizeof(resp));
    if (at_send_wait_ex("AT+CPSI?", "OK", "ERROR", CAVLI_AT_TIMEOUT_MS,
                        resp, sizeof(resp)) == ESP_OK) {
        char band[sizeof(g_diag.band)] = {0};
        cavli_parse_band(resp, band, sizeof(band));
        if (band[0]) {
            xSemaphoreTake(g_diag_mutex, portMAX_DELAY);
            strlcpy(g_diag.band, band, sizeof(g_diag.band));
            xSemaphoreGive(g_diag_mutex);
        }
    }

    cavli_diag_t diag_snapshot;
    cavli_lte_mgr_get_diag(&diag_snapshot);
    ESP_LOGI(TAG,
             "[CAVLI] Diag refresh: net=%s sim=%s csq=%d rssi=%d rsrp=%d rsrq=%d apn=%s ip=%s band=%s mqtt_code=%d",
             diag_snapshot.operator_name[0] ? diag_snapshot.operator_name : "--",
             diag_snapshot.sim_ready ? "READY" : "NOT_READY",
             (int)diag_snapshot.signal_rssi,
             (int)diag_snapshot.rssi_dbm,
             (int)diag_snapshot.rsrp_dbm,
             (int)diag_snapshot.rsrq_db,
             diag_snapshot.apn[0] ? diag_snapshot.apn : "--",
             diag_snapshot.ip_addr[0] ? diag_snapshot.ip_addr : "--",
             diag_snapshot.band[0] ? diag_snapshot.band : "--",
             (int)diag_snapshot.mqtt_reason_code);
}

static void cavli_diag_refresh_task(void *arg)
{
    cavli_refresh_diag_once();
    g_diag_refresh_task_handle = NULL;
    vTaskDelete(NULL);
}

/* ============================================================================
 * Internal helpers — UART / AT
 * ========================================================================= */

/**
 * at_send_wait_ex — send an AT command and wait for one of two expected strings.
 *
 * UART sharing strategy:
 *   Holds g_uart_mutex for the full send+read cycle. The UART RX task (Block 4)
 *   tries to acquire the same mutex with a 50 ms timeout and yields while a
 *   command is in flight — preventing it from consuming AT responses.
 *
 * +CME ERROR handling:
 *   If "+CME ERROR:" is found, the detail string is captured into
 *   g_diag.error_detail and ESP_FAIL is returned.
 *
 * @param cmd          AT command string (without \r — appended internally)
 * @param expected1    Primary substring to match in the modem response
 * @param expected2    Secondary substring (e.g. "+CEREG: 0,5"); NULL = unused
 * @param timeout_ms   How long to wait
 * @param out_buf      Optional: receives raw response before sanitisation (for parsing)
 * @param out_size     Size of out_buf; ignored if out_buf is NULL
 * @return ESP_OK on match, ESP_ERR_TIMEOUT on timeout, ESP_FAIL on CME error
 */
static esp_err_t at_send_wait_ex(const char *cmd,
                                  const char *expected1, const char *expected2,
                                  uint32_t timeout_ms,
                                  char *out_buf, size_t out_size)
{
    if (xSemaphoreTake(g_uart_mutex, pdMS_TO_TICKS(timeout_ms + 500)) != pdTRUE) {
        ESP_LOGW(TAG, "at_send_wait: mutex timeout for cmd '%s'", cmd);
        return ESP_ERR_TIMEOUT;
    }

    // Flush any stale bytes in the RX ring buffer before sending
    uart_flush_input((uart_port_t)CAVLI_UART_NUM);

    // Send command + CR (Cavli uses \r as line terminator)
    ESP_LOGD(TAG, ">> %s", cmd);
    uart_write_bytes((uart_port_t)CAVLI_UART_NUM, cmd, strlen(cmd));
    uart_write_bytes((uart_port_t)CAVLI_UART_NUM, "\r", 1);

    // Accumulate response until expected string found or timeout
    char     rx_buf[512];
    int      rx_total = 0;
    memset(rx_buf, 0, sizeof(rx_buf));

    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms);

    while (xTaskGetTickCount() < deadline) {
        esp_task_wdt_reset();
        uint8_t chunk[64];
        int n = uart_read_bytes((uart_port_t)CAVLI_UART_NUM, chunk,
                                sizeof(chunk), pdMS_TO_TICKS(20));
        if (n > 0) {
            // Clamp to remaining buffer space
            int space = (int)sizeof(rx_buf) - 1 - rx_total;
            if (n > space) n = space;
            memcpy(rx_buf + rx_total, chunk, n);
            rx_total += n;
            rx_buf[rx_total] = '\0';

            // Found expected1 or expected2
            bool matched = strstr(rx_buf, expected1) != NULL;
            if (!matched && expected2) matched = strstr(rx_buf, expected2) != NULL;

            if (matched) {
                // Copy raw response for caller before sanitising
                if (out_buf && out_size > 0) {
                    int copy_len = rx_total < (int)(out_size - 1) ? rx_total : (int)(out_size - 1);
                    memcpy(out_buf, rx_buf, copy_len);
                    out_buf[copy_len] = '\0';
                }
                // Sanitise \r/\n → '|' for readable log (Cavli uses \r as terminator)
                for (int i = 0; i < rx_total; i++) {
                    if (rx_buf[i] == '\r' || rx_buf[i] == '\n') rx_buf[i] = '|';
                }
                ESP_LOGD(TAG, "<< %s", rx_buf);
                xSemaphoreGive(g_uart_mutex);
                return ESP_OK;
            }

            // Found a +CME ERROR — capture detail string and abort
            char *cme = strstr(rx_buf, "+CME ERROR:");
            if (cme) {
                char *end = strpbrk(cme, "\r\n");
                size_t len = end ? (size_t)(end - cme) : strlen(cme);
                if (len >= sizeof(g_diag.error_detail)) {
                    len = sizeof(g_diag.error_detail) - 1;
                }
                xSemaphoreTake(g_diag_mutex, portMAX_DELAY);
                memcpy(g_diag.error_detail, cme, len);
                g_diag.error_detail[len] = '\0';
                xSemaphoreGive(g_diag_mutex);

                for (int i = 0; i < rx_total; i++) {
                    if (rx_buf[i] == '\r' || rx_buf[i] == '\n') rx_buf[i] = '|';
                }
                ESP_LOGD(TAG, "<< %s", rx_buf);
                ESP_LOGW(TAG, "CME error on '%s': %s", cmd, g_diag.error_detail);
                xSemaphoreGive(g_uart_mutex);
                return ESP_FAIL;
            }
        }
    }

    // Log whatever partial response arrived before timeout
    for (int i = 0; i < rx_total; i++) {
        if (rx_buf[i] == '\r' || rx_buf[i] == '\n') rx_buf[i] = '|';
    }
    ESP_LOGD(TAG, "<< (timeout) %s", rx_total > 0 ? rx_buf : "(no response)");
    ESP_LOGW(TAG, "at_send_wait: timeout waiting for '%s' — cmd was: %s",
             expected1, cmd);
    xSemaphoreGive(g_uart_mutex);
    return ESP_ERR_TIMEOUT;
}

/* Thin wrapper — single expected string, no response capture */
static esp_err_t at_send_wait(const char *cmd, const char *expected, uint32_t timeout_ms)
{
    return at_send_wait_ex(cmd, expected, NULL, timeout_ms, NULL, 0);
}

/**
 * uart_read_until — accumulate UART bytes until sub1 or sub2 found, or timeout.
 * MUST be called with g_uart_mutex already held by the caller.
 *
 * @param sub1        Primary substring to match → returns ESP_OK
 * @param sub2        Secondary substring (error/fail string); NULL = unused → returns ESP_FAIL
 * @param timeout_ms  How long to wait
 * @param out_buf     Optional: filled with accumulated bytes (useful for PUBLISH FAIL check)
 * @param out_size    Size of out_buf; ignored if out_buf is NULL
 * @return ESP_OK if sub1 found, ESP_FAIL if sub2 found, ESP_ERR_TIMEOUT on timeout
 */
static esp_err_t uart_read_until(const char *sub1, const char *sub2,
                                  uint32_t timeout_ms,
                                  char *out_buf, size_t out_size)
{
    char    rx_buf[512];
    int     rx_total = 0;
    memset(rx_buf, 0, sizeof(rx_buf));

    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms);

    while (xTaskGetTickCount() < deadline) {
        esp_task_wdt_reset();
        uint8_t chunk[64];
        int n = uart_read_bytes((uart_port_t)CAVLI_UART_NUM, chunk,
                                sizeof(chunk), pdMS_TO_TICKS(20));
        if (n > 0) {
            int space = (int)sizeof(rx_buf) - 1 - rx_total;
            if (n > space) n = space;
            memcpy(rx_buf + rx_total, chunk, n);
            rx_total += n;
            rx_buf[rx_total] = '\0';

            if (strstr(rx_buf, sub1) != NULL) {
                if (out_buf && out_size > 0) {
                    int copy_len = rx_total < (int)(out_size - 1) ? rx_total : (int)(out_size - 1);
                    memcpy(out_buf, rx_buf, copy_len);
                    out_buf[copy_len] = '\0';
                }
                return ESP_OK;
            }
            if (sub2 && strstr(rx_buf, sub2) != NULL) {
                if (out_buf && out_size > 0) {
                    int copy_len = rx_total < (int)(out_size - 1) ? rx_total : (int)(out_size - 1);
                    memcpy(out_buf, rx_buf, copy_len);
                    out_buf[copy_len] = '\0';
                }
                return ESP_FAIL;
            }
        }
    }
    return ESP_ERR_TIMEOUT;
}

/**
 * cavli_publish_raw — two-step AT+MQTTPUBLM publish sequence.
 * Acquires g_uart_mutex for the full exchange (command → '>' → payload → PUBLISH SUCCESS).
 * Used internally for both birth message (retain=1) and normal publishes.
 *
 * @param topic        Full topic string
 * @param payload      Payload bytes (need not be null-terminated)
 * @param payload_len  Exact byte count to send — modem auto-publishes after receiving this many bytes
 * @param qos          QoS (0 or 1)
 * @param retain       Retain flag (0 or 1)
 * @return ESP_OK on PUBLISH SUCCESS; ESP_FAIL on PUBLISH FAIL or timeout
 */
static esp_err_t cavli_publish_raw(const char *topic, const char *payload,
                                    int payload_len, int qos, int retain)
{
    if (g_mqtt_token < 0) {
        ESP_LOGW(TAG, "cavli_publish_raw: no MQTT token (not connected)");
        return ESP_FAIL;
    }

    // Format: AT+MQTTPUBLM=<token>,"<topic>",<qos>,<dup>,<retain>,<len>
    char cmd[256];
    int cmd_len = snprintf(cmd, sizeof(cmd),
                           "AT+MQTTPUBLM=%d,\"%s\",%d,0,%d,%d",
                           g_mqtt_token, topic, qos, retain, payload_len);
    if (cmd_len >= (int)sizeof(cmd)) {
        ESP_LOGW(TAG, "cavli_publish_raw: topic too long (topic=%s)", topic);
        return ESP_FAIL;
    }

    if (xSemaphoreTake(g_uart_mutex, pdMS_TO_TICKS(10000)) != pdTRUE) {
        ESP_LOGW(TAG, "cavli_publish_raw: mutex timeout (topic=%s)", topic);
        return ESP_ERR_TIMEOUT;
    }

    uart_flush_input((uart_port_t)CAVLI_UART_NUM);

    // Step 1: Send AT+MQTTPUBLM command
    ESP_LOGD(TAG, ">> %s  [payload %d bytes]", cmd, payload_len);
    uart_write_bytes((uart_port_t)CAVLI_UART_NUM, cmd, strlen(cmd));
    uart_write_bytes((uart_port_t)CAVLI_UART_NUM, "\r", 1);

    // Step 2: Wait for ">" prompt (3 s)
    esp_err_t ret = uart_read_until(">", NULL, 3000, NULL, 0);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "cavli_publish_raw: no '>' prompt (topic=%s)", topic);
        xSemaphoreGive(g_uart_mutex);
        return ESP_FAIL;
    }

    // Step 3: Send exactly payload_len bytes — no Ctrl+Z; modem auto-publishes at byte count
    uart_write_bytes((uart_port_t)CAVLI_UART_NUM, payload, payload_len);

    // Step 4: Wait for PUBLISH SUCCESS (or PUBLISH FAIL) — 5 s
    ret = uart_read_until("PUBLISH SUCCESS", "PUBLISH FAIL", 5000, NULL, 0);
    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGW(TAG, "cavli_publish_raw: PUBLISH FAIL (topic=%s)", topic);
        } else {
            ESP_LOGW(TAG, "cavli_publish_raw: timeout waiting PUBLISH SUCCESS (topic=%s)", topic);
        }
        xSemaphoreGive(g_uart_mutex);
        return ESP_FAIL;
    }

    ESP_LOGD(TAG, "<< PUBLISH SUCCESS (%s, %d bytes)", topic, payload_len);
    xSemaphoreGive(g_uart_mutex);
    return ESP_OK;
}

/* ============================================================================
 * Internal helpers — UART RX task + URC dispatch (Block 4)
 * ========================================================================= */

/**
 * cavli_process_line — parse one null-terminated modem URC line and dispatch.
 * Called from cavli_uart_rx_task AFTER g_uart_mutex has been released.
 */
static void cavli_process_line(char *line, int len)
{
    if (len == 0) return;

    /* ----- +MQTTPUBLISH: <token>,<qos>,<topic>,<len>,<payload> ----- */
    if (strncmp(line, "+MQTTPUBLISH:", 13) == 0) {
        char *p = line + 13;
        while (*p == ' ') p++;

        // Skip token field
        char *comma = strchr(p, ',');
        if (!comma) return;
        p = comma + 1;

        // Skip qos field
        comma = strchr(p, ',');
        if (!comma) return;
        p = comma + 1;

        // Parse topic — unquoted or quoted
        char topic[96] = {0};
        if (*p == '"') {
            p++;
            int ti = 0;
            while (*p && *p != '"' && ti < (int)sizeof(topic) - 1) topic[ti++] = *p++;
            if (*p == '"') p++;
        } else {
            int ti = 0;
            while (*p && *p != ',' && ti < (int)sizeof(topic) - 1) topic[ti++] = *p++;
        }
        if (*p != ',') return;
        p++;

        // Parse declared payload length (for logging)
        char *end_ptr;
        long payload_len = strtol(p, &end_ptr, 10);
        if (*end_ptr != ',') return;
        char *payload = end_ptr + 1;  // payload starts here; null-terminated by line_buf

        ESP_LOGI(TAG, "[CAVLI] Incoming: topic=%s  len=%ld  payload=%s", topic, payload_len, payload);

        // Route to config callback if topic matches
        if (strcmp(topic, g_config_topic) == 0 && g_config_cb) {
            g_config_cb(payload);   // payload is already null-terminated at line_buf end
        }
        return;
    }

    /* ----- +MQTTCONN: <token>: CONNECTED[,<reason>]  (Tier-1 auto-reconnect) -----
     * Guard against matching "CONNECTING" by requiring the char after CONNECTED
     * to be ',' or end-of-string, never 'I'. */
    if (strstr(line, "+MQTTCONN:")) {
        char *cp = strstr(line, ": CONNECTED");
        if (cp && cp[11] != 'I') {
            // ": CONNECTED" not followed by 'I' → genuine CONNECTED, not CONNECTING
            ESP_LOGI(TAG, "[CAVLI] URC: MQTT re-connected (Tier-1 auto)");
            xSemaphoreTake(g_diag_mutex, portMAX_DELAY);
            bool was_connected = g_diag.connected;
            g_diag.connected = true;
            g_diag.state     = CAVLI_STATE_CONNECTED;
            g_diag.mqtt_reason_code = cavli_parse_reason_code(line);
            g_diag.consecutive_publish_failures = 0;
            if (!was_connected) g_diag.mqtt_reconnects++;
            snprintf(g_diag.last_recovery_action,
                     sizeof(g_diag.last_recovery_action), "MQTT_AUTO_RECONNECT");
            xSemaphoreGive(g_diag_mutex);
            // Re-publish birth so broker shows "online" immediately
            // QoS 0: broker doesn't ACK QoS 1 reliably after Tier-1 auto-reconnect;
            // retain=1 ensures broker keeps the value if it lands.
            const char *birth = "online";
            cavli_publish_raw(g_lwt_topic, birth, (int)strlen(birth), 0, 1);
            // Flush any statechange/reason JSON missed during the connectivity gap
            json_logger_flush_pending();
            return;
        }
        if (strstr(line, ": DISCONNECTED")) {
            ESP_LOGW(TAG, "[CAVLI] URC: MQTT disconnected — Tier-1 auto-reconnect active");
            xSemaphoreTake(g_diag_mutex, portMAX_DELAY);
            g_diag.connected = false;
            g_diag.state     = CAVLI_STATE_DISCONNECTED;
            g_diag.mqtt_reason_code = cavli_parse_reason_code(line);
            xSemaphoreGive(g_diag_mutex);
            cavli_wake_reconnect_task();
            return;
        }
    }

    /* ----- +MQTTDISCONNECTED: <token>  (broker keepalive timeout variant) ----- */
    if (strncmp(line, "+MQTTDISCONNECTED:", 18) == 0) {
        ESP_LOGW(TAG, "[CAVLI] URC: MQTT disconnected (broker timeout)");
        xSemaphoreTake(g_diag_mutex, portMAX_DELAY);
        g_diag.connected = false;
        g_diag.state     = CAVLI_STATE_DISCONNECTED;
        g_diag.mqtt_reason_code = cavli_parse_reason_code(line);
        xSemaphoreGive(g_diag_mutex);
        cavli_wake_reconnect_task();
        return;
    }

    // Everything else (OK, ERROR, CONNECTING, etc.) — debug-level only
    ESP_LOGD(TAG, "[RX] %s", line);
}

/**
 * cavli_uart_rx_task — continuously reads UART2 and dispatches URC lines.
 *
 * Mutex-yield strategy (prevents consuming AT command responses):
 *   Tries to acquire g_uart_mutex with a 50 ms timeout each iteration.
 *   While at_send_wait_ex or cavli_publish_raw holds the mutex (command in
 *   flight), this task yields without reading — bytes stay in the ring buffer.
 *   When the mutex is free, reads available bytes, releases mutex immediately,
 *   then processes any complete \r\n-terminated lines.
 *
 * Uses a 4 KB static line buffer (BSS, not stack) so the task itself only
 * needs a small stack.
 */
static void cavli_uart_rx_task(void *arg)
{
    static char line_buf[4096];
    static int  line_len = 0;
    line_len = 0;
    memset(line_buf, 0, sizeof(line_buf));

    while (!g_stop_requested) {
        // Yield while an AT command is in flight
        if (xSemaphoreTake(g_uart_mutex, pdMS_TO_TICKS(50)) != pdTRUE) {
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }

        uint8_t chunk[128];
        // Non-blocking read (timeout=0): mutex held for microseconds, not 20 ms.
        // This prevents starvation of the connect task which runs at priority 3.
        int n = uart_read_bytes((uart_port_t)CAVLI_UART_NUM, chunk,
                                sizeof(chunk), 0);
        xSemaphoreGive(g_uart_mutex);   // release ASAP — don't hold during processing

        if (n <= 0) {
            vTaskDelay(pdMS_TO_TICKS(5));   // short yield when nothing to read
            continue;
        }

        // Accumulate bytes; dispatch on \r or \n
        for (int i = 0; i < n; i++) {
            char c = (char)chunk[i];
            if (c == '\r' || c == '\n') {
                if (line_len > 0) {
                    line_buf[line_len] = '\0';
                    cavli_process_line(line_buf, line_len);
                    line_len = 0;
                }
            } else {
                if (line_len < (int)sizeof(line_buf) - 1) {
                    line_buf[line_len++] = c;
                }
            }
        }
    }

    g_rx_task_handle = NULL;   // Signal stop() that we have exited
    vTaskDelete(NULL);
}

/* ============================================================================
 * Internal helpers — network init
 * ========================================================================= */

/**
 * cavli_connect_once — full network + MQTT init sequence (steps 0–15).
 *
 * Step 0    : Blind-blast AT+MQTTDELETE=3,4,5 (clears zombie handles on ESP32 restart)
 * Steps 1-3 : AT liveness; AT+TRB reset only if modem unresponsive
 * Steps 4-5 : ATE0, AT+CMEE=2 (non-critical)
 * Step 6    : AT+CFUN=1 (radio on)
 * Step 7    : AT+CPIN? (SIM check, 3 retries)
 * Steps 7b-c: AT+GSN (IMEI), AT+CNUM (SIM phone number) — log only
 * Step 8    : AT+CEREG? polling (up to 60 s, accepts home=1 or roaming=5)
 * Step 9    : AT+CSQ (signal strength → g_diag.signal_rssi)
 * Step 10   : AT+CGDCONT (PDP context, empty APN = operator default)
 * Step 11   : AT+CGACT (activate PDP, retry once)
 * Step 12   : AT+MQTTCREATE (creates MQTT client, returns token, retry once)
 * Step 13   : AT+MQTTCONN (connects to broker, 30 s timeout, retry once)
 * Step 14   : Publish birth "online" to /lwt, QoS 1, retain=1 (non-fatal)
 * Step 15   : AT+MQTTSUBUNSUB to config topic, QoS 1 (non-fatal)
 *
 * g_diag.state is updated at every step and frozen at the failure point on abort.
 * g_diag.connected is set true and state = CONNECTED only after all steps complete.
 * Returns ESP_OK when fully operational; ESP_FAIL on any unrecoverable error.
 */
static esp_err_t cavli_connect_once(void)
{
    esp_err_t ret;

    /* ------------------------------------------------------------------
     * Step 0: Blind-blast MQTT handle cleanup
     * Responses ignored — modem returns an error for empty slots (harmless).
     * Covers the case where ESP32 restarted and g_mqtt_token reset to -1
     * but the modem still holds old handles from the previous run.
     * ---------------------------------------------------------------- */
    ESP_LOGI(TAG, "[CAVLI] connect: clearing MQTT handles 3,4,5");
    // Accept OK (slot had a client) or ERROR (slot was empty) — both are terminal responses
    at_send_wait_ex("AT+MQTTDELETE=3", "OK", "ERROR", 1000, NULL, 0);
    at_send_wait_ex("AT+MQTTDELETE=4", "OK", "ERROR", 1000, NULL, 0);
    at_send_wait_ex("AT+MQTTDELETE=5", "OK", "ERROR", 1000, NULL, 0);
    xSemaphoreTake(g_diag_mutex, portMAX_DELAY);
    g_mqtt_token = -1;
    xSemaphoreGive(g_diag_mutex);

    /* ------------------------------------------------------------------
     * Step 1: AT liveness (3 retries × 500 ms)
     * ---------------------------------------------------------------- */
    UPDATE_STATE(CAVLI_STATE_AT_LIVENESS);
    bool liveness_ok = false;
    for (int i = 0; i < 3 && !liveness_ok; i++) {
        if (at_send_wait("AT", "OK", 500) == ESP_OK) {
            liveness_ok = true;
        } else {
            vTaskDelay(pdMS_TO_TICKS(500));
        }
    }

    if (!liveness_ok) {
        /* Step 2: hardware reset — only triggered when modem is unresponsive */
        UPDATE_STATE(CAVLI_STATE_HW_RESET);
        ESP_LOGW(TAG, "[CAVLI] AT unresponsive — sending AT+TRB hardware reset");
        at_send_wait("AT+TRB", "OK", 5000);    // ignore result
        vTaskDelay(pdMS_TO_TICKS(3000));        // wait for modem to reboot

        /* Step 3: re-liveness after reset */
        UPDATE_STATE(CAVLI_STATE_AT_LIVENESS);
        for (int i = 0; i < 3 && !liveness_ok; i++) {
            if (at_send_wait("AT", "OK", 1000) == ESP_OK) {
                liveness_ok = true;
            } else {
                vTaskDelay(pdMS_TO_TICKS(500));
            }
        }
        if (!liveness_ok) {
            SET_ABORT(CAVLI_ERR_AT_NO_RESPONSE);
            return ESP_FAIL;
        }
    }

    /* ------------------------------------------------------------------
     * Step 4: ATE0 — disable echo for clean UART parsing
     * Step 5: AT+CMEE=2 — verbose error strings
     * Non-critical: log warning on fail, continue.
     * ---------------------------------------------------------------- */
    UPDATE_STATE(CAVLI_STATE_ECHO_OFF);
    ret = at_send_wait("ATE0", "OK", CAVLI_AT_TIMEOUT_MS);
    if (ret != ESP_OK) ESP_LOGW(TAG, "[CAVLI] ATE0 failed — continuing");

    UPDATE_STATE(CAVLI_STATE_ERROR_REPORTING);
    ret = at_send_wait("AT+CMEE=2", "OK", CAVLI_AT_TIMEOUT_MS);
    if (ret != ESP_OK) ESP_LOGW(TAG, "[CAVLI] AT+CMEE=2 failed — CME errors may be numeric");

    /* ------------------------------------------------------------------
     * Step 6: AT+CFUN=1 — enable full radio functionality
     * ---------------------------------------------------------------- */
    UPDATE_STATE(CAVLI_STATE_RADIO_ON);
    ret = at_send_wait("AT+CFUN=1", "OK", CAVLI_AT_TIMEOUT_MS);
    if (ret != ESP_OK) ESP_LOGW(TAG, "[CAVLI] AT+CFUN=1 failed — radio may already be on");

    /* ------------------------------------------------------------------
     * Step 7: AT+CPIN? — SIM check (3 retries × 2 s)
     * ---------------------------------------------------------------- */
    UPDATE_STATE(CAVLI_STATE_SIM_CHECK);
    bool sim_ok = false;
    for (int i = 0; i < 3 && !sim_ok; i++) {
        if (at_send_wait("AT+CPIN?", "+CPIN: READY", CAVLI_AT_TIMEOUT_MS) == ESP_OK) {
            sim_ok = true;
        } else if (i < 2) {
            ESP_LOGW(TAG, "[CAVLI] SIM not ready (attempt %d/3) — retrying in 2s", i + 1);
            vTaskDelay(pdMS_TO_TICKS(2000));
        }
    }
    if (!sim_ok) {
        SET_ABORT(CAVLI_ERR_SIM_NOT_READY);
        return ESP_FAIL;
    }

    /* ------------------------------------------------------------------
     * Step 7b: AT+CGSN — IMEI (log only)
     * Step 7c: AT+CNUM — SIM phone number (log only)
     * Placed after SIM check so CNUM has an accessible SIM.
     * ---------------------------------------------------------------- */
    {
        char resp[64] = {0};
        if (at_send_wait_ex("AT+GSN", "OK", "ERROR", CAVLI_AT_TIMEOUT_MS,
                             resp, sizeof(resp)) == ESP_OK) {
            // IMEI is the 15-digit numeric string in the response
            char *p = resp;
            while (*p) {
                if (*p >= '0' && *p <= '9') {
                    int len = 0;
                    char *start = p;
                    while (start[len] >= '0' && start[len] <= '9') len++;
                    if (len == 15) {
                        char imei[16]; memcpy(imei, start, 15); imei[15] = '\0';
                        ESP_LOGI(TAG, "[CAVLI] IMEI        : %s", imei);
                        break;
                    }
                    p += (len > 0 ? len : 1);
                } else {
                    p++;
                }
            }
        }
    }
    {
        char resp[64] = {0};
        if (at_send_wait_ex("AT+CNUM", "OK", NULL, CAVLI_AT_TIMEOUT_MS,
                             resp, sizeof(resp)) == ESP_OK) {
            // Response: +CNUM: "","<number>",<type>   OR just OK (not stored on SIM)
            char *cnum = strstr(resp, "+CNUM:");
            if (cnum) {
                char *first_comma = strchr(cnum, ',');  // comma after alpha-tag field
                if (first_comma && first_comma[1] == '"') {
                    char *num_start = first_comma + 2;  // skip ,"
                    char *num_end   = strchr(num_start, '"');
                    if (num_end && num_end > num_start) {
                        int len = (int)(num_end - num_start);
                        xSemaphoreTake(g_diag_mutex, portMAX_DELAY);
                        if (len >= (int)sizeof(g_diag.sim_phone))
                            len = (int)sizeof(g_diag.sim_phone) - 1;
                        memcpy(g_diag.sim_phone, num_start, len);
                        g_diag.sim_phone[len] = '\0';
                        xSemaphoreGive(g_diag_mutex);
                        ESP_LOGI(TAG, "[CAVLI] SIM phone   : %s", g_diag.sim_phone);
                    }
                }
            } else {
                ESP_LOGI(TAG, "[CAVLI] SIM phone   : not stored on SIM");
            }
        }
    }

    /* ------------------------------------------------------------------
     * Step 8: AT+CEREG? — network registration polling (up to 60 s)
     * Accepts stat=1 (home) or stat=5 (roaming).
     * ---------------------------------------------------------------- */
    UPDATE_STATE(CAVLI_STATE_NET_REGISTRATION);
    TickType_t reg_deadline = xTaskGetTickCount() + pdMS_TO_TICKS(CAVLI_NET_REG_TIMEOUT_MS);
    bool registered = false;
    while (xTaskGetTickCount() < reg_deadline && !registered) {
        ret = at_send_wait_ex("AT+CEREG?", "+CEREG: 0,1", "+CEREG: 0,5",
                               CAVLI_AT_TIMEOUT_MS, NULL, 0);
        if (ret == ESP_OK) {
            registered = true;
        } else {
            ESP_LOGI(TAG, "[CAVLI] Not registered yet — retrying");
            vTaskDelay(pdMS_TO_TICKS(2000));
        }
    }
    if (!registered) {
        SET_ABORT(CAVLI_ERR_NET_REG_TIMEOUT);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "[CAVLI] Network registered");

    /* ------------------------------------------------------------------
     * Step 9: AT+CSQ — signal strength → g_diag.signal_rssi
     * ---------------------------------------------------------------- */
    UPDATE_STATE(CAVLI_STATE_SIGNAL_CHECK);
    {
        char csq_resp[64] = {0};
        if (at_send_wait_ex("AT+CSQ", "+CSQ:", NULL, CAVLI_AT_TIMEOUT_MS,
                             csq_resp, sizeof(csq_resp)) == ESP_OK) {
            char *p = strstr(csq_resp, "+CSQ:");
            if (p) {
                p += 5;                         // skip "+CSQ:"
                while (*p == ' ') p++;          // skip spaces
                int rssi = (int)strtol(p, NULL, 10);
                xSemaphoreTake(g_diag_mutex, portMAX_DELAY);
                g_diag.signal_rssi = (int8_t)rssi;
                g_diag.rssi_dbm = cavli_csq_to_dbm(rssi);
                xSemaphoreGive(g_diag_mutex);
                const char *quality = (rssi == 99) ? "unknown"
                                    : (rssi >= 20)  ? "good"
                                    : (rssi >= 10)  ? "fair" : "weak";
                ESP_LOGI(TAG, "[CAVLI] Signal RSSI=%d (%s)", rssi, quality);
            }
        }
    }

    /* ------------------------------------------------------------------
     * Step 10: AT+CGDCONT — set PDP context (Jio APN: jionet)
     * ---------------------------------------------------------------- */
    UPDATE_STATE(CAVLI_STATE_PDP_CONTEXT);
    ret = at_send_wait("AT+CGDCONT=1,\"IP\",\"jionet\"", "OK", CAVLI_AT_TIMEOUT_MS);
    if (ret != ESP_OK) ESP_LOGW(TAG, "[CAVLI] AT+CGDCONT failed — using existing PDP context");

    /* ------------------------------------------------------------------
     * Step 11: AT+CGACT=1,1 — activate PDP context (retry once after 3 s)
     * ---------------------------------------------------------------- */
    UPDATE_STATE(CAVLI_STATE_PDP_ACTIVATE);
    ret = at_send_wait("AT+CGACT=1,1", "OK", CAVLI_AT_TIMEOUT_MS);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "[CAVLI] AT+CGACT first attempt failed — retrying in 3s");
        vTaskDelay(pdMS_TO_TICKS(3000));
        ret = at_send_wait("AT+CGACT=1,1", "OK", CAVLI_AT_TIMEOUT_MS);
    }
    if (ret != ESP_OK) {
        SET_ABORT(CAVLI_ERR_PDP_FAILED);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "[CAVLI] Network ready — PDP context active");

    /* ------------------------------------------------------------------
     * Step 12: AT+MQTTCREATE — create MQTT client, receive integer handle token
     * 12-param form: broker, port, clientid, keepalive, cleansession,
     *   user, pass, lwt_topic, lwt_msg, lwt_qos, lwt_retain, mqtt_version
     * ---------------------------------------------------------------- */
    ret = cavli_sync_system_time_from_modem();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "[CAVLI] Continuing without LTE time sync");
    }

    UPDATE_STATE(CAVLI_STATE_MQTT_CREATE);
    g_mqtt_token = -1;

    char create_cmd[512];
    snprintf(create_cmd, sizeof(create_cmd),
             "AT+MQTTCREATE=\"%s\",%u,\"%s\",%u,1,\"%s\",\"%s\",\"%s\",\"offline\",1,1,4",
             g_broker, (unsigned)g_port, g_client_id, (unsigned)g_keepalive,
             g_username, g_password, g_lwt_topic);

    char create_resp[128] = {0};
    ret = at_send_wait_ex(create_cmd, "CREATED", NULL, CAVLI_AT_TIMEOUT_MS,
                          create_resp, sizeof(create_resp));
    // Parse token from "+MQTTCREATE: <n>: CREATED"
    if (ret == ESP_OK) {
        char *p = strstr(create_resp, "+MQTTCREATE:");
        if (p) {
            p += (int)strlen("+MQTTCREATE:");
            while (*p == ' ') p++;
            long token = strtol(p, NULL, 10);
            if (token >= 0) g_mqtt_token = (int)token;
        }
    }
    if (ret != ESP_OK || g_mqtt_token < 0) {
        ESP_LOGW(TAG, "[CAVLI] AT+MQTTCREATE attempt 1 failed — retrying in 2 s");
        vTaskDelay(pdMS_TO_TICKS(2000));
        memset(create_resp, 0, sizeof(create_resp));
        g_mqtt_token = -1;
        ret = at_send_wait_ex(create_cmd, "CREATED", NULL, CAVLI_AT_TIMEOUT_MS,
                              create_resp, sizeof(create_resp));
        if (ret == ESP_OK) {
            char *p = strstr(create_resp, "+MQTTCREATE:");
            if (p) {
                p += (int)strlen("+MQTTCREATE:");
                while (*p == ' ') p++;
                long token = strtol(p, NULL, 10);
                if (token >= 0) g_mqtt_token = (int)token;
            }
        }
    }
    if (ret != ESP_OK || g_mqtt_token < 0) {
        SET_ABORT(CAVLI_ERR_MQTT_CREATE_FAILED);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "[CAVLI] MQTT handle token  : %d", g_mqtt_token);

    /* ------------------------------------------------------------------
     * Step 13: AT+MQTTCONN — connect to broker
     * reconnection_flag=1 → Cavli Tier-1 auto-reconnect on drop
     * Timeout: CAVLI_CONN_TIMEOUT_MS (30 s)
     * ---------------------------------------------------------------- */
    UPDATE_STATE(CAVLI_STATE_MQTT_CONNECT);
    char conn_cmd[64];
    snprintf(conn_cmd, sizeof(conn_cmd),
             "AT+MQTTCONN=%d,1,%u", g_mqtt_token, (unsigned)g_keepalive);
    ret = at_send_wait_ex(conn_cmd, "CONNECTED", NULL, CAVLI_CONN_TIMEOUT_MS, NULL, 0);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "[CAVLI] AT+MQTTCONN attempt 1 failed — retrying in 2 s");
        vTaskDelay(pdMS_TO_TICKS(2000));
        ret = at_send_wait_ex(conn_cmd, "CONNECTED", NULL, CAVLI_CONN_TIMEOUT_MS, NULL, 0);
    }
    if (ret != ESP_OK) {
        SET_ABORT(CAVLI_ERR_MQTT_CONNECT_TIMEOUT);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "[CAVLI] MQTT connected to %s:%u", g_broker, g_port);

    /* ------------------------------------------------------------------
     * Step 14: Birth message — publish "online" to /lwt, QoS 1, retain=1
     * Non-fatal: broker will show stale LWT if this fails, but device continues.
     * ---------------------------------------------------------------- */
    UPDATE_STATE(CAVLI_STATE_MQTT_BIRTH);
    {
        // QoS 0: no PUBACK required — avoids timeout when broker session state
        // is not yet stable after fresh connect. retain=1 ensures broker keeps it.
        const char *birth = "online";
        ret = cavli_publish_raw(g_lwt_topic, birth, (int)strlen(birth), 0, 1);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "[CAVLI] Birth message failed — broker may retain stale LWT");
        } else {
            ESP_LOGI(TAG, "[CAVLI] Birth 'online' published → %s", g_lwt_topic);
        }
    }

    /* ------------------------------------------------------------------
     * Step 15: Config topic subscribe — QoS 1
     * Non-fatal: incoming config won't work but outbound publish is unaffected.
     * ---------------------------------------------------------------- */
    UPDATE_STATE(CAVLI_STATE_MQTT_SUBSCRIBE);
    {
        char sub_cmd[200];
        snprintf(sub_cmd, sizeof(sub_cmd),
                 "AT+MQTTSUBUNSUB=%d,\"%s\",1,0", g_mqtt_token, g_config_topic);
        ret = at_send_wait_ex(sub_cmd, "SUBSCRIBE SUCCESS", NULL, CAVLI_AT_TIMEOUT_MS, NULL, 0);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "[CAVLI] Config subscribe failed — incoming config won't work");
        } else {
            ESP_LOGI(TAG, "[CAVLI] Subscribed to config topic: %s", g_config_topic);
        }
    }

    /* All steps complete — mark fully operational */
    xSemaphoreTake(g_diag_mutex, portMAX_DELAY);
    g_diag.connected  = true;
    g_diag.state      = CAVLI_STATE_CONNECTED;
    g_diag.last_error = CAVLI_ERR_NONE;
    g_diag.mqtt_reason_code = 0;
    xSemaphoreGive(g_diag_mutex);
    ESP_LOGI(TAG, "[CAVLI] Fully operational — client_id=%s  token=%d",
             g_client_id, g_mqtt_token);
    cavli_lte_mgr_refresh_diag_async();
    return ESP_OK;
}

/* ============================================================================
 * Public API
 * ========================================================================= */

esp_err_t cavli_lte_mgr_init(const char *broker_host, uint16_t broker_port,
                               const char *client_id,   const char *username,
                               const char *password,    const char *base_topic,
                               uint16_t keepalive)
{
    if (!broker_host || broker_host[0] == '\0' ||
        !client_id || client_id[0] == '\0' ||
        !base_topic || base_topic[0] == '\0') {
        ESP_LOGE(TAG, "Invalid Cavli init args");
        return ESP_ERR_INVALID_ARG;
    }
    if (g_uart_mutex || g_diag_mutex || g_rx_task_handle || g_reconnect_task_handle) {
        ESP_LOGE(TAG, "Cavli manager already initialized/running");
        return ESP_ERR_INVALID_STATE;
    }

    // Store all parameters into static globals
    strlcpy(g_broker,     broker_host, sizeof(g_broker));
    strlcpy(g_client_id,  client_id,   sizeof(g_client_id));
    strlcpy(g_username,   username ? username : "", sizeof(g_username));
    strlcpy(g_password,   password ? password : "", sizeof(g_password));
    strlcpy(g_base_topic, base_topic, sizeof(g_base_topic));
    g_port      = broker_port;
    g_keepalive = keepalive;

    // Build derived topic strings
    snprintf(g_lwt_topic,        sizeof(g_lwt_topic),        "%s/lwt",        g_base_topic);
    snprintf(g_config_topic,     sizeof(g_config_topic),     "%s/config",     g_base_topic);
    snprintf(g_config_ack_topic, sizeof(g_config_ack_topic), "%s/config/ack", g_base_topic);

    // Create mutexes
    g_uart_mutex = xSemaphoreCreateMutex();
    g_diag_mutex = xSemaphoreCreateMutex();
    if (!g_uart_mutex || !g_diag_mutex) {
        ESP_LOGE(TAG, "Failed to create mutexes");
        if (g_uart_mutex) {
            vSemaphoreDelete(g_uart_mutex);
            g_uart_mutex = NULL;
        }
        if (g_diag_mutex) {
            vSemaphoreDelete(g_diag_mutex);
            g_diag_mutex = NULL;
        }
        return ESP_ERR_NO_MEM;
    }

    memset(&g_diag, 0, sizeof(g_diag));
    cavli_diag_set_defaults(&g_diag);

    ESP_LOGI(TAG, "Init: broker=%s:%u  client_id=%s  keepalive=%us",
             g_broker, g_port, g_client_id, g_keepalive);
    ESP_LOGI(TAG, "Topics: lwt=%s  config=%s", g_lwt_topic, g_config_topic);

    return ESP_OK;
}

/* ============================================================================
 * Reconnect task (Block 6) — handles both the initial connect attempt and all
 * subsequent Tier 2 retries with exponential backoff.
 *
 * Tier 1: Cavli modem auto-reconnects internally (reconnection_flag=1 in
 *         AT+MQTTCONN). Detected by RX task via +MQTTCONN: CONNECTED URC.
 * Tier 2: This task. Fires when g_diag.connected is still false after the
 *         backoff period — covers boot failure, modem crash, Tier 1 exhausted.
 *
 * Backoff: immediate → 60 s → 120 s → 300 s (cap). Resets to 60 s on success.
 * Exit: g_stop_requested flag set by cavli_lte_mgr_stop().
 * Stack: 8 KB (cavli_connect_once uses large AT command buffers on stack).
 * ========================================================================= */
static void cavli_reconnect_task(void *arg)
{
    uint32_t backoff_ms = 0;   // 0 = run immediately on first iteration
    uint32_t full_failures_since_reset = 0;
    uint32_t last_modem_reset_ms = 0;
    esp_task_wdt_add(NULL);

    while (!g_stop_requested) {
        esp_task_wdt_reset();
        // Wait before each attempt; first pass (backoff_ms == 0) runs immediately.
        // Slice into 1-second chunks so g_stop_requested is honoured quickly.
        if (backoff_ms > 0) {
            uint32_t slices = backoff_ms / 1000;
            for (uint32_t i = 0; i < slices && !g_stop_requested; i++) {
                esp_task_wdt_reset();
                if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000)) > 0) {
                    ESP_LOGI(TAG, "[CAVLI] Reconnect wait interrupted by connection event");
                    break;
                }
            }
        }
        if (g_stop_requested) break;

        xSemaphoreTake(g_diag_mutex, portMAX_DELAY);
        bool connected = g_diag.connected;
        xSemaphoreGive(g_diag_mutex);

        if (!connected) {
            xSemaphoreTake(g_diag_mutex, portMAX_DELAY);
            g_diag.state = CAVLI_STATE_RECONNECTING;
            g_diag.connect_attempts++;
            uint32_t attempt = g_diag.connect_attempts;
            xSemaphoreGive(g_diag_mutex);

            if (backoff_ms == 0) {
                ESP_LOGI(TAG, "[CAVLI] Initial connect attempt");
            } else {
                ESP_LOGI(TAG, "[CAVLI] Reconnect attempt #%lu (waited %lu s)",
                         (unsigned long)attempt, (unsigned long)(backoff_ms / 1000));
            }

            esp_task_wdt_reset();
            esp_err_t ret = cavli_connect_once();
            esp_task_wdt_reset();
            if (ret == ESP_OK) {
                ESP_LOGI(TAG, "[CAVLI] Connect attempt #%lu: PASS — fully operational",
                         (unsigned long)attempt);
                backoff_ms = CAVLI_RECONNECT_BACKOFF_INITIAL_MS;  // reset; next check in 60 s
                full_failures_since_reset = 0;
                xSemaphoreTake(g_diag_mutex, portMAX_DELAY);
                g_diag.consecutive_publish_failures = 0;
                snprintf(g_diag.last_recovery_action,
                         sizeof(g_diag.last_recovery_action), "FULL_RECONNECT");
                xSemaphoreGive(g_diag_mutex);
            } else {
                full_failures_since_reset++;
                // Advance backoff: 0→60 s→120 s→300 s (cap)
                backoff_ms = (backoff_ms == 0)
                    ? CAVLI_RECONNECT_BACKOFF_INITIAL_MS
                    : ((backoff_ms * 2 < CAVLI_RECONNECT_BACKOFF_MAX_MS)
                           ? backoff_ms * 2
                           : CAVLI_RECONNECT_BACKOFF_MAX_MS);

                ESP_LOGE(TAG,
                         "[CAVLI] Attempt #%lu FAILED at %s (%s). Retry in %lu s",
                         (unsigned long)attempt,
                         cavli_state_to_str(g_diag.state),
                         cavli_error_to_str(g_diag.last_error),
                         (unsigned long)(backoff_ms / 1000));

                uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
                bool reset_allowed = last_modem_reset_ms == 0 ||
                    (uint32_t)(now_ms - last_modem_reset_ms) >=
                        CAVLI_MODEM_RESET_COOLDOWN_MS;
                if (full_failures_since_reset >=
                        CAVLI_FULL_FAILURES_BEFORE_RESET && reset_allowed) {
                    ESP_LOGW(TAG, "[CAVLI] Three full reconnect failures; restarting modem");
                    (void)cavli_lte_mgr_restart_modem();
                    last_modem_reset_ms = now_ms;
                    full_failures_since_reset = 0;
                    backoff_ms = CAVLI_RECONNECT_BACKOFF_INITIAL_MS;
                    vTaskDelay(pdMS_TO_TICKS(3000));
                }
            }
        } else {
            // Already connected — wake up every 60 s to check again (Tier 2 standby)
            if (backoff_ms == 0) {
                backoff_ms = CAVLI_RECONNECT_BACKOFF_INITIAL_MS;
            }
        }
    }

    esp_task_wdt_delete(NULL);
    g_reconnect_task_handle = NULL;  // Signal stop() that we have exited
    vTaskDelete(NULL);
}

esp_err_t cavli_lte_mgr_start(void)
{
    if (!g_uart_mutex || !g_diag_mutex) {
        ESP_LOGE(TAG, "Cavli manager not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    if (g_rx_task_handle || g_reconnect_task_handle) {
        ESP_LOGW(TAG, "Cavli manager already started");
        return ESP_OK;
    }

    // ------------------------------------------------------------------
    // 1. UART2 hardware init (one-time — not repeated on reconnect)
    // ------------------------------------------------------------------
    const uart_config_t uart_cfg = {
        .baud_rate  = CAVLI_UART_BAUD_RATE,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t ret;

    ret = uart_param_config((uart_port_t)CAVLI_UART_NUM, &uart_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "uart_param_config failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = uart_set_pin((uart_port_t)CAVLI_UART_NUM,
                       CAVLI_UART_TX_GPIO, CAVLI_UART_RX_GPIO,
                       UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "uart_set_pin failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = uart_driver_install((uart_port_t)CAVLI_UART_NUM,
                              CAVLI_UART_RX_BUF_SIZE,  // RX ring buffer
                              0,                        // TX: 0 = synchronous
                              0, NULL, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "uart_driver_install failed: %s", esp_err_to_name(ret));
        return ret;
    }
    g_uart_driver_installed = true;

    ESP_LOGI(TAG, "UART2 ready: TX=GPIO%d  RX=GPIO%d  baud=%d",
             CAVLI_UART_TX_GPIO, CAVLI_UART_RX_GPIO, CAVLI_UART_BAUD_RATE);

    // ------------------------------------------------------------------
    // 2. Start UART RX task — must launch before the connect task so that
    //    any URCs that arrive after MQTTCONN are not lost.
    // ------------------------------------------------------------------
    BaseType_t rc = xTaskCreatePinnedToCore(cavli_uart_rx_task, "cavli_rx",
                                             8192, NULL, 2, &g_rx_task_handle, 0);
    if (rc != pdPASS) {
        ESP_LOGE(TAG, "Failed to create cavli_rx_task — out of memory");
        uart_driver_delete((uart_port_t)CAVLI_UART_NUM);
        g_uart_driver_installed = false;
        return ESP_ERR_NO_MEM;
    }

    // ------------------------------------------------------------------
    // 3. Start reconnect task (8 KB stack, priority 3, CPU0).
    //    Handles the initial connect immediately, then retries in the
    //    background with exponential backoff if disconnected.
    //    app_main always proceeds — connectivity managed in background.
    // ------------------------------------------------------------------
    rc = xTaskCreatePinnedToCore(cavli_reconnect_task, "cavli_reconnect",
                                  8192, NULL, 3, &g_reconnect_task_handle, 0);
    if (rc != pdPASS) {
        ESP_LOGE(TAG, "Failed to create cavli_reconnect_task — out of memory");
        g_stop_requested = true;
        while (g_rx_task_handle != NULL) {
            vTaskDelay(pdMS_TO_TICKS(50));
        }
        uart_driver_delete((uart_port_t)CAVLI_UART_NUM);
        g_uart_driver_installed = false;
        g_stop_requested = false;
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}

/* ============================================================================
 * cavli_lte_mgr_stop (Block 6)
 *
 * Signals both background tasks to exit, waits for them to confirm (each sets
 * its handle to NULL before calling vTaskDelete), then performs a graceful
 * MQTT disconnect and full UART2 cleanup.
 *
 * NOTE: If the reconnect task is mid-way through cavli_connect_once() when
 * stop() is called, this function will block until the current AT sequence
 * finishes (up to ~90 s worst case). This is acceptable — stop() is only
 * called on deliberate shutdown or uplink-mode switch.
 * ========================================================================= */
void cavli_lte_mgr_stop(void)
{
    ESP_LOGI(TAG, "[CAVLI] stop() — signalling tasks to exit");
    g_stop_requested = true;

    // Wait for reconnect task to notice the flag and exit (checks every 1 s during backoff).
    // If it is inside cavli_connect_once(), this may take up to ~90 s.
    while (g_reconnect_task_handle != NULL) {
        vTaskDelay(pdMS_TO_TICKS(200));
    }

    // Wait for RX task to exit (notices flag within ~50 ms of next mutex acquire).
    while (g_rx_task_handle != NULL) {
        vTaskDelay(pdMS_TO_TICKS(200));
    }

    while (g_diag_refresh_task_handle != NULL) {
        vTaskDelay(pdMS_TO_TICKS(200));
    }

    ESP_LOGI(TAG, "[CAVLI] All tasks exited — cleaning up modem state");

    if (g_uart_driver_installed) {
        // Graceful MQTT disconnect (best-effort; we now exclusively own UART2).
        if (g_mqtt_token >= 0) {
            char cmd[32];
            snprintf(cmd, sizeof(cmd), "AT+MQTTDISCONN=%d", g_mqtt_token);
            at_send_wait(cmd, "OK", 3000);  // result ignored
        }

        // Blind-blast MQTT delete for tokens 3, 4, 5 - leaves modem clean for next boot.
        at_send_wait("AT+MQTTDELETE=3", "OK", 1000);
        at_send_wait("AT+MQTTDELETE=4", "OK", 1000);
        at_send_wait("AT+MQTTDELETE=5", "OK", 1000);
    }
    g_mqtt_token = -1;

    // Deinit UART2 driver.
    if (g_uart_driver_installed) {
        uart_driver_delete((uart_port_t)CAVLI_UART_NUM);
        g_uart_driver_installed = false;
        ESP_LOGI(TAG, "[CAVLI] UART2 deinitialized");
    }

    // Delete synchronisation objects.
    if (g_uart_mutex) {
        vSemaphoreDelete(g_uart_mutex);
        g_uart_mutex = NULL;
    }
    if (g_diag_mutex) {
        vSemaphoreDelete(g_diag_mutex);
        g_diag_mutex = NULL;
    }

    // Reset all state — allows cavli_lte_mgr_init() + cavli_lte_mgr_start() to be called again.
    memset(&g_diag, 0, sizeof(g_diag));
    cavli_diag_set_defaults(&g_diag);
    g_stop_requested = false;
    ESP_LOGI(TAG, "[CAVLI] Stopped cleanly");
}

esp_err_t cavli_lte_mgr_restart_modem(void)
{
    if (!g_uart_driver_installed || g_uart_mutex == NULL) {
        ESP_LOGW(TAG, "[CAVLI] User restart skipped: modem manager is not active");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGW(TAG, "[CAVLI] User-requested modem restart (AT+TRB)");
    esp_err_t ret = at_send_wait("AT+TRB", "OK", 5000);
    xSemaphoreTake(g_diag_mutex, portMAX_DELAY);
    g_diag.modem_restarts++;
    snprintf(g_diag.last_recovery_action,
             sizeof(g_diag.last_recovery_action), "MODEM_RESTART");
    xSemaphoreGive(g_diag_mutex);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "[CAVLI] AT+TRB acknowledgement not received: %s",
                 esp_err_to_name(ret));
    }
    return ret;
}

bool cavli_lte_mgr_is_connected(void)
{
    if (!g_diag_mutex) return false;
    xSemaphoreTake(g_diag_mutex, portMAX_DELAY);
    bool c = g_diag.connected;
    xSemaphoreGive(g_diag_mutex);
    return c;
}

esp_err_t cavli_lte_mgr_publish(const char *topic, const char *payload, int qos)
{
    if (!topic || !payload) return ESP_ERR_INVALID_ARG;
    if (!g_diag_mutex) return ESP_ERR_INVALID_STATE;
    // Guard: only publish when MQTT is connected
    xSemaphoreTake(g_diag_mutex, portMAX_DELAY);
    bool connected = g_diag.connected;
    xSemaphoreGive(g_diag_mutex);

    if (!connected) {
        xSemaphoreTake(g_diag_mutex, portMAX_DELAY);
        g_diag.messages_failed++;
        xSemaphoreGive(g_diag_mutex);
        return ESP_FAIL;
    }

    int payload_len = (int)strlen(payload);
    if (payload_len > CAVLI_MAX_PAYLOAD_SIZE) {
        ESP_LOGW(TAG, "cavli_lte_mgr_publish: payload too large (%d > %d)",
                 payload_len, CAVLI_MAX_PAYLOAD_SIZE);
        xSemaphoreTake(g_diag_mutex, portMAX_DELAY);
        g_diag.messages_failed++;
        xSemaphoreGive(g_diag_mutex);
        return ESP_FAIL;
    }

    esp_err_t ret = cavli_publish_raw(topic, payload, payload_len, qos, 0);

    bool wake_reconnect = false;
    xSemaphoreTake(g_diag_mutex, portMAX_DELAY);
    if (ret == ESP_OK) {
        g_diag.messages_sent++;
        g_diag.consecutive_publish_failures = 0;
    } else {
        g_diag.messages_failed++;
        g_diag.last_error = CAVLI_ERR_PUBLISH_FAILED;
        if (g_diag.consecutive_publish_failures < UINT32_MAX) {
            g_diag.consecutive_publish_failures++;
        }
        if (g_diag.consecutive_publish_failures >= CAVLI_PUBLISH_FAILURE_LIMIT) {
            g_diag.connected = false;
            g_diag.state = CAVLI_STATE_DISCONNECTED;
            snprintf(g_diag.last_recovery_action,
                     sizeof(g_diag.last_recovery_action), "PUBLISH_FAILURE");
            wake_reconnect = true;
        }
    }
    xSemaphoreGive(g_diag_mutex);

    if (wake_reconnect) {
        ESP_LOGW(TAG, "[CAVLI] Three consecutive publish failures; reconnect requested");
        cavli_wake_reconnect_task();
    }

    return ret;
}

void cavli_lte_mgr_set_config_callback(cavli_config_cb_t cb)
{
    g_config_cb = cb;
}

esp_err_t cavli_lte_mgr_publish_config_ack(const char *config_id,
                                             const char *catalog_version,
                                             const char *status,
                                             const char *error)
{
    if (!g_diag_mutex) return ESP_ERR_INVALID_STATE;
    xSemaphoreTake(g_diag_mutex, portMAX_DELAY);
    bool connected = g_diag.connected;
    xSemaphoreGive(g_diag_mutex);

    if (!connected) return ESP_ERR_INVALID_STATE;

    // Build ISO8601 UTC timestamp
    char ts[32] = {0};
    time_t now = time(NULL);
    struct tm tm_info;
    gmtime_r(&now, &tm_info);
    strftime(ts, sizeof(ts), "%Y-%m-%dT%H:%M:%SZ", &tm_info);

    // Build JSON payload — mirrors mqtt_client_mgr_publish_config_ack() schema
    cJSON *root = cJSON_CreateObject();
    if (!root) return ESP_ERR_NO_MEM;

    cJSON_AddStringToObject(root, "msg_type",        "config.ack");
    cJSON_AddStringToObject(root, "config_id",       config_id       ? config_id       : "");
    cJSON_AddStringToObject(root, "catalog_version", catalog_version ? catalog_version : "");
    cJSON_AddStringToObject(root, "status",          status          ? status          : "");
    cJSON_AddStringToObject(root, "ts",              ts);
    if (error && error[0] != '\0') {
        cJSON_AddStringToObject(root, "error", error);
    }

    char *json_str = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    if (!json_str) return ESP_ERR_NO_MEM;

    esp_err_t ret = cavli_lte_mgr_publish(g_config_ack_topic, json_str, 1);
    cJSON_free(json_str);
    return ret;
}

void cavli_lte_mgr_get_diag(cavli_diag_t *out)
{
    if (!out) return;
    if (!g_diag_mutex) {
        memset(out, 0, sizeof(*out));
        cavli_diag_set_defaults(out);
        return;
    }
    xSemaphoreTake(g_diag_mutex, portMAX_DELAY);
    *out = g_diag;
    xSemaphoreGive(g_diag_mutex);
}

esp_err_t cavli_lte_mgr_refresh_diag_async(void)
{
    if (!g_diag_mutex || !g_uart_mutex) {
        return ESP_ERR_INVALID_STATE;
    }
    if (g_diag_refresh_task_handle != NULL) {
        return ESP_OK;
    }
    BaseType_t rc = xTaskCreatePinnedToCore(cavli_diag_refresh_task,
                                            "cavli_diag_refresh",
                                            6144, NULL, 2,
                                            &g_diag_refresh_task_handle, 0);
    if (rc != pdPASS) {
        g_diag_refresh_task_handle = NULL;
        ESP_LOGE(TAG, "Failed to create cavli_diag_refresh_task");
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

const char *cavli_state_to_str(cavli_state_t s)
{
    switch (s) {
        case CAVLI_STATE_IDLE:              return "IDLE";
        case CAVLI_STATE_AT_LIVENESS:       return "AT_LIVENESS";
        case CAVLI_STATE_HW_RESET:          return "HW_RESET";
        case CAVLI_STATE_ECHO_OFF:          return "ECHO_OFF";
        case CAVLI_STATE_ERROR_REPORTING:   return "ERROR_REPORTING";
        case CAVLI_STATE_RADIO_ON:          return "RADIO_ON";
        case CAVLI_STATE_SIM_CHECK:         return "SIM_CHECK";
        case CAVLI_STATE_NET_REGISTRATION:  return "NET_REGISTRATION";
        case CAVLI_STATE_SIGNAL_CHECK:      return "SIGNAL_CHECK";
        case CAVLI_STATE_PDP_CONTEXT:       return "PDP_CONTEXT";
        case CAVLI_STATE_PDP_ACTIVATE:      return "PDP_ACTIVATE";
        case CAVLI_STATE_MQTT_CREATE:       return "MQTT_CREATE";
        case CAVLI_STATE_MQTT_CONNECT:      return "MQTT_CONNECT";
        case CAVLI_STATE_MQTT_BIRTH:        return "MQTT_BIRTH";
        case CAVLI_STATE_MQTT_SUBSCRIBE:    return "MQTT_SUBSCRIBE";
        case CAVLI_STATE_CONNECTED:         return "CONNECTED";
        case CAVLI_STATE_DISCONNECTED:      return "DISCONNECTED";
        case CAVLI_STATE_RECONNECTING:      return "RECONNECTING";
        default:                            return "UNKNOWN";
    }
}

const char *cavli_error_to_str(cavli_error_t e)
{
    switch (e) {
        case CAVLI_ERR_NONE:                 return "NONE";
        case CAVLI_ERR_AT_NO_RESPONSE:       return "AT_NO_RESPONSE";
        case CAVLI_ERR_SIM_NOT_READY:        return "SIM_NOT_READY";
        case CAVLI_ERR_NET_REG_TIMEOUT:      return "NET_REG_TIMEOUT";
        case CAVLI_ERR_PDP_FAILED:           return "PDP_FAILED";
        case CAVLI_ERR_MQTT_CREATE_FAILED:   return "MQTT_CREATE_FAILED";
        case CAVLI_ERR_MQTT_CONNECT_TIMEOUT: return "MQTT_CONNECT_TIMEOUT";
        case CAVLI_ERR_PUBLISH_FAILED:       return "PUBLISH_FAILED";
        case CAVLI_ERR_PUBLISH_TIMEOUT:      return "PUBLISH_TIMEOUT";
        default:                             return "UNKNOWN";
    }
}
