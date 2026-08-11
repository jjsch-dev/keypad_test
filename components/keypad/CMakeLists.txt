idf_component_register(SRCS "src/keypad.c" "src/keypad_gtx312l.c"
                       INCLUDE_DIRS "include"
                       PRIV_INCLUDE_DIRS "src"
                       REQUIRES esp_driver_i2c esp_driver_gpio esp_timer)
