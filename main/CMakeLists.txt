# ============================================================================
#  Sediment Tray Controller  ·  FW v1.3.0
#  ESP-IDF component CMakeLists  ·  single-file firmware
# ============================================================================
idf_component_register(
    SRCS "sensor_hub.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES
        esp_driver_gpio
        esp_driver_uart
        esp_driver_ledc
        esp_driver_pcnt
        esp_driver_mcpwm
        nvs_flash
        esp_timer
        esp_system
        esp_rom
        freertos
        log
        soc
)


# Bench test (encoder by hand): idf.py -DSHUB_BENCH_TEST=1 build
# Cached by CMake - turn off again with: idf.py -DSHUB_BENCH_TEST=0 build
if(SHUB_BENCH_TEST)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE SHUB_BENCH_TEST=1)
endif()
