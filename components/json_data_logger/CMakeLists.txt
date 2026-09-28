idf_component_register(
    SRCS "json_data_logger.c"
    INCLUDE_DIRS "include"
    REQUIRES freertos espressif__cjson esp_timer nvs_config log log_config
)
