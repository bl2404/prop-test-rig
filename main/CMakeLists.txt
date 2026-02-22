idf_component_register(SRCS "main.c"
                    REQUIRES dshot_esc
                    PRIV_REQUIRES esp_driver_rmt esp_adc
                    INCLUDE_DIRS ".")
