idf_component_register(
    SRCS "keypad_pcf8574_matrix.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES esp_timer driver i2c_mux_mgr log_config
)
