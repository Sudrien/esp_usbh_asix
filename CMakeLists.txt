idf_component_register(
    SRCS "esp_usbh_asix.c"
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "priv_include"
    REQUIRES usb iot_eth
)
