# Uses driver/i2c_master.h (ESP-IDF v5.2+). The legacy driver/i2c.h API is
# end-of-life and disappears in v7.0, so this is the right side of that line.
idf_component_register(SRCS "color_sensor.c" INCLUDE_DIRS "."
                       REQUIRES esp_driver_i2c)
