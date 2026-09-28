idf_component_register(
    SRCS "datameter_lcd.c"
    INCLUDE_DIRS "."
    REQUIRES u8g2
    PRIV_REQUIRES 
        driver
        esp_driver_gpio
        esp_timer
        esp_rom
        freertos
        log
)
