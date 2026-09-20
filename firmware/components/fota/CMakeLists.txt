idf_component_register(
    SRCS "fota.c"
    INCLUDE_DIRS "include"
    REQUIRES canopen robocar_od app_update esp_app_format esp_system
)
