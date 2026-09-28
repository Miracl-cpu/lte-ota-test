/**
 * @file setup_softap.c
 * @brief SoftAP management for setup mode implementation
 *
 * Creates an open WiFi access point with captive portal DNS.
 */

#include "setup_softap.h"
#include "provisioning_mgr.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "log_config.h"
#undef LOG_LOCAL_LEVEL
#define LOG_LOCAL_LEVEL LOG_LEVEL_SETUP_SOFTAP
#include "esp_log.h"
#include "esp_mac.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "setup_softap";

// Internal state
static bool s_softap_active = false;
static bool s_dns_active = false;
static char s_softap_ssid[20] = {0};
static esp_netif_t *s_ap_netif = NULL;
static esp_netif_t *s_sta_netif = NULL;
static TaskHandle_t s_dns_task_handle = NULL;
static int s_dns_socket = -1;

// DNS server configuration
#define DNS_PORT        53
#define DNS_MAX_LEN     512

// DNS header structure
typedef struct __attribute__((packed)) {
    uint16_t id;
    uint16_t flags;
    uint16_t qdcount;
    uint16_t ancount;
    uint16_t nscount;
    uint16_t arcount;
} dns_header_t;

/**
 * @brief WiFi event handler for SoftAP
 */
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                                int32_t event_id, void *event_data)
{
    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t *event = (wifi_event_ap_staconnected_t *)event_data;
        ESP_LOGI(TAG, "Station connected: MAC=" MACSTR ", AID=%d",
                 MAC2STR(event->mac), event->aid);
    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t *event = (wifi_event_ap_stadisconnected_t *)event_data;
        ESP_LOGI(TAG, "Station disconnected: MAC=" MACSTR ", AID=%d",
                 MAC2STR(event->mac), event->aid);
    }
}

/**
 * @brief Build DNS response for captive portal
 * Returns a response pointing all queries to the SoftAP IP
 */
static int build_dns_response(uint8_t *request, int req_len, uint8_t *response, int max_len)
{
    if (req_len < sizeof(dns_header_t) || max_len < req_len + 16) {
        return -1;
    }

    // Copy the request as base for response
    memcpy(response, request, req_len);
    dns_header_t *hdr = (dns_header_t *)response;

    // Set response flags: QR=1 (response), AA=1 (authoritative), no error
    hdr->flags = htons(0x8400);
    hdr->ancount = htons(1);

    // Find end of question section
    int pos = sizeof(dns_header_t);
    while (pos < req_len && response[pos] != 0) {
        pos += response[pos] + 1;
    }
    pos += 5;  // Skip null terminator + QTYPE (2) + QCLASS (2)

    // Add answer
    // Name pointer to question
    response[pos++] = 0xC0;
    response[pos++] = 0x0C;

    // Type A (1)
    response[pos++] = 0x00;
    response[pos++] = 0x01;

    // Class IN (1)
    response[pos++] = 0x00;
    response[pos++] = 0x01;

    // TTL (60 seconds)
    response[pos++] = 0x00;
    response[pos++] = 0x00;
    response[pos++] = 0x00;
    response[pos++] = 0x3C;

    // Data length (4 for IPv4)
    response[pos++] = 0x00;
    response[pos++] = 0x04;

    // IP address: 192.168.4.1
    response[pos++] = 192;
    response[pos++] = 168;
    response[pos++] = 4;
    response[pos++] = 1;

    return pos;
}

/**
 * @brief DNS server task for captive portal
 */
static void dns_server_task(void *arg)
{
    struct sockaddr_in server_addr, client_addr;
    socklen_t client_len = sizeof(client_addr);
    uint8_t rx_buffer[DNS_MAX_LEN];
    uint8_t tx_buffer[DNS_MAX_LEN];

    // Create UDP socket
    s_dns_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (s_dns_socket < 0) {
        ESP_LOGE(TAG, "Failed to create DNS socket: errno %d", errno);
        s_dns_active = false;
        vTaskDelete(NULL);
        return;
    }

    // Bind to DNS port
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    server_addr.sin_port = htons(DNS_PORT);

    if (bind(s_dns_socket, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        ESP_LOGE(TAG, "Failed to bind DNS socket: errno %d", errno);
        close(s_dns_socket);
        s_dns_socket = -1;
        s_dns_active = false;
        vTaskDelete(NULL);
        return;
    }

    // Set socket timeout
    struct timeval timeout;
    timeout.tv_sec = 1;
    timeout.tv_usec = 0;
    setsockopt(s_dns_socket, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

    ESP_LOGI(TAG, "DNS server started on port %d", DNS_PORT);

    while (s_dns_active) {
        int len = recvfrom(s_dns_socket, rx_buffer, sizeof(rx_buffer), 0,
                          (struct sockaddr *)&client_addr, &client_len);

        if (len > 0) {
            ESP_LOGD(TAG, "DNS query received (%d bytes)", len);

            // Build and send response
            int resp_len = build_dns_response(rx_buffer, len, tx_buffer, sizeof(tx_buffer));
            if (resp_len > 0) {
                sendto(s_dns_socket, tx_buffer, resp_len, 0,
                       (struct sockaddr *)&client_addr, client_len);
                ESP_LOGD(TAG, "DNS response sent (%d bytes)", resp_len);
            }
        }
    }

    // Cleanup
    if (s_dns_socket >= 0) {
        close(s_dns_socket);
        s_dns_socket = -1;
    }

    ESP_LOGI(TAG, "DNS server stopped");
    s_dns_task_handle = NULL;
    vTaskDelete(NULL);
}

esp_err_t setup_softap_start(char *ssid_out)
{
    if (s_softap_active) {
        ESP_LOGW(TAG, "SoftAP already active");
        if (ssid_out) {
            strncpy(ssid_out, s_softap_ssid, 19);
        }
        return ESP_OK;
    }

    esp_err_t ret;

    // Initialize netif if needed
    ret = esp_netif_init();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "Failed to init netif: %s", esp_err_to_name(ret));
        return ret;
    }

    // Create default event loop if needed
    ret = esp_event_loop_create_default();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "Failed to create event loop: %s", esp_err_to_name(ret));
        return ret;
    }

    // Create AP netif (or reuse existing one)
    if (s_ap_netif == NULL) {
        // Check if AP netif already exists (may have been created elsewhere)
        s_ap_netif = esp_netif_get_handle_from_ifkey("WIFI_AP_DEF");
        if (s_ap_netif == NULL) {
            s_ap_netif = esp_netif_create_default_wifi_ap();
            if (s_ap_netif == NULL) {
                ESP_LOGE(TAG, "Failed to create AP netif");
                return ESP_FAIL;
            }
            ESP_LOGI(TAG, "Created new AP netif");
        } else {
            ESP_LOGI(TAG, "Using existing AP netif");
        }
    }

    // Create STA netif (required for WiFi scanning in APSTA mode)
    if (s_sta_netif == NULL) {
        // Check if STA netif already exists (may have been created by wifi_mgr)
        s_sta_netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
        if (s_sta_netif == NULL) {
            s_sta_netif = esp_netif_create_default_wifi_sta();
            if (s_sta_netif == NULL) {
                ESP_LOGW(TAG, "Failed to create STA netif (scan may not work)");
                // Continue anyway - AP will still work
            } else {
                ESP_LOGI(TAG, "Created new STA netif");
            }
        } else {
            ESP_LOGI(TAG, "Using existing STA netif");
        }
    }

    // Initialize WiFi
    wifi_init_config_t wifi_init_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ret = esp_wifi_init(&wifi_init_cfg);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "Failed to init WiFi: %s", esp_err_to_name(ret));
        return ret;
    }

    // Register event handler
    esp_event_handler_instance_t wifi_handler;
    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                        &wifi_event_handler, NULL, &wifi_handler);

    // Get MAC address for SSID
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_SOFTAP);
    snprintf(s_softap_ssid, sizeof(s_softap_ssid), "%s%02X%02X",
             SOFTAP_SSID_PREFIX, mac[4], mac[5]);

    // Configure AP
    wifi_config_t wifi_config = {
        .ap = {
            .ssid_len = strlen(s_softap_ssid),
            .channel = SOFTAP_CHANNEL,
            .max_connection = SOFTAP_MAX_CONN,
            .authmode = WIFI_AUTH_OPEN,  // Open network for easy setup
            .pmf_cfg = {
                .required = false,
            },
        },
    };
    strncpy((char *)wifi_config.ap.ssid, s_softap_ssid, sizeof(wifi_config.ap.ssid));

    // Set mode to APSTA (AP + STA) to enable WiFi scanning while AP is running
    ret = esp_wifi_set_mode(WIFI_MODE_APSTA);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set APSTA mode: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = esp_wifi_set_config(WIFI_IF_AP, &wifi_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set AP config: %s", esp_err_to_name(ret));
        return ret;
    }

    // Clear STA config to prevent autoconnect during setup mode
    wifi_config_t sta_cfg = {0};
    ret = esp_wifi_set_config(WIFI_IF_STA, &sta_cfg);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to clear STA config: %s", esp_err_to_name(ret));
        // Continue anyway
    }

    // Start WiFi
    ret = esp_wifi_start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start WiFi: %s", esp_err_to_name(ret));
        return ret;
    }

    s_softap_active = true;

    ESP_LOGI(TAG, "SoftAP started (APSTA mode): SSID='%s', IP=%s", s_softap_ssid, SOFTAP_IP);

    if (ssid_out) {
        strncpy(ssid_out, s_softap_ssid, 19);
        ssid_out[19] = '\0';
    }

    return ESP_OK;
}

esp_err_t setup_softap_stop(void)
{
    if (!s_softap_active) {
        return ESP_OK;
    }

    // Stop DNS server first
    setup_softap_stop_dns();

    // Stop WiFi
    esp_err_t ret = esp_wifi_stop();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to stop WiFi: %s", esp_err_to_name(ret));
    }

    ret = esp_wifi_deinit();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to deinit WiFi: %s", esp_err_to_name(ret));
    }

    s_softap_active = false;
    ESP_LOGI(TAG, "SoftAP stopped");

    return ESP_OK;
}

bool setup_softap_is_active(void)
{
    return s_softap_active;
}

uint8_t setup_softap_get_client_count(void)
{
    if (!s_softap_active) {
        return 0;
    }

    wifi_sta_list_t sta_list;
    if (esp_wifi_ap_get_sta_list(&sta_list) == ESP_OK) {
        return sta_list.num;
    }
    return 0;
}

const char* setup_softap_get_ip(void)
{
    return SOFTAP_IP;
}

esp_err_t setup_softap_start_dns(void)
{
    if (s_dns_active) {
        return ESP_OK;
    }

    s_dns_active = true;

    BaseType_t ret = xTaskCreate(dns_server_task, "dns_server", 4096, NULL, 5, &s_dns_task_handle);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create DNS task");
        s_dns_active = false;
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t setup_softap_stop_dns(void)
{
    if (!s_dns_active) {
        return ESP_OK;
    }

    s_dns_active = false;

    // Close socket to unblock recvfrom
    if (s_dns_socket >= 0) {
        close(s_dns_socket);
        s_dns_socket = -1;
    }

    // Wait for task to exit
    if (s_dns_task_handle != NULL) {
        vTaskDelay(pdMS_TO_TICKS(200));
    }

    ESP_LOGI(TAG, "DNS server stopped");
    return ESP_OK;
}
