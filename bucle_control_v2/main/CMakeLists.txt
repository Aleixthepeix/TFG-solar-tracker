idf_component_register(SRCS "homing.c" "main.c" "control.c" "stepper_fas.cpp" "cam_client.c"
                            "wifi.c" "console.c" "kinematics.c" "cmd_server.c"
                    INCLUDE_DIRS "."
                    REQUIRES esp_system
                             nvs_flash
                             esp_wifi
                             esp_event
                             esp_netif
                             esp_http_client
                             esp_http_server
                             console
                             driver
                             FastAccelStepper)
