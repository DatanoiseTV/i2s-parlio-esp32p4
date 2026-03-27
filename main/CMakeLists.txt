idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES parlio_i2s esp_driver_parlio esp_driver_i2s esp_hw_support
)
