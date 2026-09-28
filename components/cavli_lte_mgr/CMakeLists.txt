idf_component_register(
    SRCS "cavli_lte_mgr.c"
    INCLUDE_DIRS "include"
    REQUIRES driver esp_driver_uart esp_common freertos log log_config espressif__cjson json_data_logger
)
