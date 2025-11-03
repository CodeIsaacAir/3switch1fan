idf_component_register(SRCS "state_firmware_update.c" "state_poweron.c" "state_machine.c" "main.c" "uart_comm.c" "network.c" "state_operational.c" "tiny-json.c"
                       PRIV_REQUIRES esp_driver_uart wifi_provisioning nvs_flash mqtt esp_wifi esp_https_ota esp_http_client esp_bootloader_format app_update
                       INCLUDE_DIRS "")

