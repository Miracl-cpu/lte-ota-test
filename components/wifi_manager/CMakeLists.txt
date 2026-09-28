idf_component_register(
    SRCS "wifi_manager.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_wifi esp_event esp_netif esp_timer nvs_flash log freertos nvs_config log_config
)
