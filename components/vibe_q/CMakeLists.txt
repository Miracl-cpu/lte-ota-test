idf_component_register(
    SRCS "src/vibe_q.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_driver_gpio esp_driver_i2c esp_timer freertos wtvb01_485 log_config
)


