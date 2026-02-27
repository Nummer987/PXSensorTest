idf_component_register(SRCS "px_zigbee.c" "main.c" "../common/zcl_utility/src/zcl_utility.c"
                    INCLUDE_DIRS "." "../common/zcl_utility/include"
                    REQUIRES driver nvs_flash esp_event
                    PRIV_REQUIRES esp_driver_uart ieee802154)