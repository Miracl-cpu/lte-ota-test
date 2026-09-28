idf_component_register(
    SRCS "mqtt_client_mgr.c"
    INCLUDE_DIRS "include"
    REQUIRES espressif__mqtt esp_event log freertos nvs_config espressif__cjson log_config json_data_logger mbedtls
)
