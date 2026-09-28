idf_component_register(
    SRCS "daq_ct_ads1115.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES esp_timer freertos driver esp_driver_gpio i2c_mux_mgr log_config
)
