idf_component_register(
    SRCS "ota_drive.c"
    INCLUDE_DIRS "."
    REQUIRES esp_http_client esp_https_ota app_update wifi_manager log_config nvs_flash nvs_config
    PRIV_REQUIRES freertos esp_timer mbedtls
)
