idf_component_register(
    SRCS "i2c_mux_mgr.c"
    INCLUDE_DIRS "."
    REQUIRES driver esp_driver_i2c log_config
)
