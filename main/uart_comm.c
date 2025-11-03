#include "uart_comm.h"
#include "driver/uart.h"
#include "hal/uart_ll.h"
#include "string.h"
#include "state_operational.h"


QueueHandle_t uart_queue = NULL;

uart_config_t uart_config = {
    .baud_rate = uart_baud_rate,
    .data_bits = uart_data_bits,
    .parity = uart_parity,
    .stop_bits = uart_stop_bits,
    .flow_ctrl = uart_flow_ctrl,
};

esp_err_t uart_init(void){
    // Configure a UART interrupt and threshold
    uart_intr_config_t uart_intr={
        .intr_enable_mask = UART_INTR_RXFIFO_TOUT | UART_INTR_RXFIFO_FULL,
        .rx_timeout_thresh = 1,
        .rxfifo_full_thresh = 50,
    };
    ESP_ERROR_CHECK(uart_driver_install(uart_port, uart_rx_tx_buffer_size, uart_rx_tx_buffer_size, 10, &uart_queue, 0));
    ESP_ERROR_CHECK(uart_param_config(uart_port, &uart_config));    // Configure the UART parameters
    ESP_ERROR_CHECK(uart_intr_config(uart_port, &uart_intr));
    ESP_ERROR_CHECK(uart_enable_rx_intr(uart_port));
    ESP_ERROR_CHECK(uart_set_pin(uart_port, tx_pin, rx_pin, -1, -1));
    uart_flush(uart_port);
    return ESP_OK;
}

esp_err_t publishStatusToMCU(uint8_t gang_datapointID, uint8_t gang_action){

    printf("[uart_comm] Publishing status to MCU: gang_datapointID: %d, gang_action: %d\n", gang_datapointID, gang_action);

    update_datapoint_status(gang_datapointID, gang_action);

    if(gang_datapointID != 0x03)
    {
        const uint8_t checksum = (0x55 + 0xaa + 0x00 + 0x06 + 0x00 + 0x05 + gang_datapointID + 0x01 + 0x00 + 0x01 + gang_action) % 256;
        const uint8_t updateDp1_Message[12] = { 0x55, /*Frame Header MSByte*/
                                                0xaa, /*Frame Header LSByte*/
                                                0x00, /*Version*/
                                                0x06, /*Command Word*/
                                                0x00, /*Data Length MSByte*/
                                                0x05, /*Data Length LSByte*/
                                                gang_datapointID, /*Data Point Index*/
                                                0x01, /*Data Type*/
                                                0x00, /*Data Length MSByte*/
                                                0x01, /*Data Length LSByte*/
                                                gang_action, /*Data Value*/
                                                checksum}; /*Checksum*/
        uint8_t bytes_sent = uart_write_bytes(uart_port, updateDp1_Message, ( sizeof(updateDp1_Message)/(sizeof(uint8_t)) ));
        if(bytes_sent != sizeof(updateDp1_Message)/(sizeof(uint8_t))){
            printf("[uart_comm] Failed to send updateDp1_Message to MCU\n");
            return ESP_FAIL;
        }
    }else if(gang_datapointID == 0x03){
        printf("[uart_comm] Publishing fan status to MCU: gang_datapointID: %d, gang_action: %d\n", gang_datapointID, gang_action);

        const uint8_t checksum = (0x55 + 0xaa + 0x00 + 0x06 + 0x00 + 0x05 + gang_datapointID + 0x04 + 0x00 + 0x01 + gang_action) % 256;
        const uint8_t updateDp1_Message[12] = { 0x55, /*Frame Header MSByte*/
                                                0xaa, /*Frame Header LSByte*/
                                                0x00, /*Version*/
                                                0x06, /*Command Word*/
                                                0x00, /*Data Length MSByte*/
                                                0x05, /*Data Length LSByte*/
                                                gang_datapointID, /*Data Point Index*/
                                                0x04, /*Data Type*/
                                                0x00, /*Data Length MSByte*/
                                                0x01, /*Data Length LSByte*/
                                                gang_action, /*Data Value*/
                                                checksum}; /*Checksum*/
        uint8_t bytes_sent = uart_write_bytes(uart_port, updateDp1_Message, (sizeof(updateDp1_Message)/(sizeof(uint8_t))));
        if(bytes_sent != sizeof(updateDp1_Message)/(sizeof(uint8_t))){
            printf("[uart_comm] Failed to send updateDp1_Message to MCU\n");
            return ESP_FAIL;
        }
    }

    return ESP_OK;
}

 