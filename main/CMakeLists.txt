idf_component_register(SRCS  "grabber.c"
                       INCLUDE_DIRS "."
                        PRIV_REQUIRES spi_flash
                        nvs_flash
                        esp_driver_uart
                        esp_driver_gpio
                        freertos
                        log)
