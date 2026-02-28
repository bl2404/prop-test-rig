idf_component_register(SRCS "main.c" "dshot_esc_encoder.c" "ring_avg.c" "tenso.c"
                    PRIV_REQUIRES esp_driver_rmt esp_adc
                    INCLUDE_DIRS ".")
