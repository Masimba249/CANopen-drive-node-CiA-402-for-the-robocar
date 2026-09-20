idf_component_register(
    SRCS "can_port.c" "motor.c"
    INCLUDE_DIRS "include"
    REQUIRES canopen robocar_od cia402 driver esp_adc
)
