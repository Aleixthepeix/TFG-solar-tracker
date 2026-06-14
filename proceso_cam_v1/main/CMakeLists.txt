idf_component_register(
    SRCS
        "main.c"
        "vision.c"
        "server.c"
    INCLUDE_DIRS "."
    REQUIRES
        espressif__esp32-camera
        nvs_flash
        esp_wifi
        esp_netif
        esp_event
        esp_http_server
)
