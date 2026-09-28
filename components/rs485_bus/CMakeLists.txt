idf_component_register(
    SRCS "rs485_bus.c"
    INCLUDE_DIRS "include"
    REQUIRES driver esp_driver_uart esp_rom freertos
)
