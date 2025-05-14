set(COMPONENT_SRCS "async.c" "conf.c" "i2c.c" "web_ota.c" "pwr.c" "uart.c" "tcp_com.c" "ns_com.c" "flash.c" "main.c" "hardware.c" "ananlog.c" "wifi.c" "bt_hid_manager.c")
set(COMPONENT_ADD_INCLUDEDIRS "./")
set(COMPONENT_EMBED_FILES
    "html/www/HttpOTA.html.gz"
    )

file(GLOB SRC_LIST "*.c" "*.cpp")


register_component()

#idf_component_register(SRCS  ${SRC_LIST} INCLUDE_DIRS ".")