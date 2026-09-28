idf_component_register(
    SRCS "avf133_m1.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_timer freertos log_config rs485_bus
)
