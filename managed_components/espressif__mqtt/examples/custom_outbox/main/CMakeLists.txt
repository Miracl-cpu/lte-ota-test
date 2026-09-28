idf_component_register(SRCS "custom_outbox.cpp" "app_main.c"
                    INCLUDE_DIRS "."
                    PRIV_REQUIRES mqtt nvs_flash esp_netif
                    )
