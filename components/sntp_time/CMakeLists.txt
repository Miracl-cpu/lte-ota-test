idf_component_register(
    SRCS "sntp_time.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_netif log freertos log_config
)
