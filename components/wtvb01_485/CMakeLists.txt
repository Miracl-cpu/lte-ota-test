idf_component_register(
    SRCS "src/wtvb01_485.c" "src/wtvb01_modbus.c"
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "src"
    REQUIRES log_config rs485_bus
)
