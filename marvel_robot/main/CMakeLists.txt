idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS ""
    REQUIRES motor ir_ultrasonic color_sensor robot_state settings web_server esp_timer
)
