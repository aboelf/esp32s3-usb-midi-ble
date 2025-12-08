idf_component_register(SRCS "main.c"
                       INCLUDE_DIRS "."
                       REQUIRES tinyusb led_strip nvs_flash bt driver)
