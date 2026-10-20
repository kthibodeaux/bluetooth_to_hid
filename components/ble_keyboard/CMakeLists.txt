idf_component_register(SRCS "ble_keyboard.c" "hid_report_map.c"
                       INCLUDE_DIRS "include"
                       PRIV_REQUIRES bt nvs_flash)
