
idf_component_register(
        SRCS "led_strip_encoder.c" "main.c" "ble_ancs_manager.c"
        "ble_ancs.c"
        PRIV_REQUIRES bt nvs_flash
        INCLUDE_DIRS "${CMAKE_CURRENT_BINARY_DIR}")
