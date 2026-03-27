idf_component_register(
    SRCS "parlio_i2s.c"
    INCLUDE_DIRS "include"
    REQUIRES driver esp_driver_parlio esp_driver_i2s esp_hw_support
)
