idf_component_register(SRCS "motor.c" INCLUDE_DIRS "."
                       REQUIRES esp_driver_gpio esp_driver_ledc)
