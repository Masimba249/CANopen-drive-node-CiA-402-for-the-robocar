idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES canopen cia402 robocar_od drive_hw fota
             nvs_flash esp_timer esp_hw_support driver
)
