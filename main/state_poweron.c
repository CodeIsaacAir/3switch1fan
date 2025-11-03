// Custom includes
#include "state_poweron.h"
#include "freertos/projdefs.h"
#include "uart_comm.h"
#include "state_common.h"
#include "driver/uart.h"

// ESP-IDF includes
#include "esp_err.h"


static TaskHandle_t heartbeatMessage_Transmission_h = NULL;
static struct state_common_data* data_lifetime;

static void heartbeatMessage_Transmission(void *pvParameters){
    while(1){
        uart_write_bytes((uart_port_t)uart_port,data_lifetime->heartbeatMessage,sizeof(data_lifetime->heartbeatMessage));
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

esp_err_t powerOn_entry(struct state_common_data* data){
    printf("[powerOn] Power on entry\n");
    data_lifetime = data;
    xTaskCreatePinnedToCore(heartbeatMessage_Transmission, "heartbeatMessage_Transmission", 2048, data_lifetime, 1, &heartbeatMessage_Transmission_h, 0);
    return ESP_OK;
}

esp_err_t powerOn_exit(struct state_common_data* data){
    if(heartbeatMessage_Transmission_h != NULL){
        vTaskDelete(heartbeatMessage_Transmission_h);
    }
    return ESP_OK;
}