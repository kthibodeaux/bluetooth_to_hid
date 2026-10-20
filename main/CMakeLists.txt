idf_component_register(SRCS "main.c" "usb_selftest.c" "bond_reset.c"
                       INCLUDE_DIRS "."
                       REQUIRES ble_keyboard usb_keyboard nvs_flash esp_driver_gpio)
