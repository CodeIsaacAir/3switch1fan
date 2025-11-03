#ifndef UART_COMM_H
#define UART_COMM_H

#include "esp_err.h"
#include "freertos/FreeRTOS.h"  // Ignore warning as it is required by freertos/queue.h
#include "freertos/queue.h"

/*defines*/
// UART config
#define uart_port UART_NUM_2
#define uart_baud_rate 9600
#define uart_data_bits UART_DATA_8_BITS
#define uart_parity UART_PARITY_DISABLE
#define uart_stop_bits UART_STOP_BITS_1
#define uart_flow_ctrl UART_HW_FLOWCTRL_DISABLE
// Rx, Tx Buffer Size
#define uart_rx_tx_buffer_size 256
// Rx, Tx pins
#define rx_pin 16
#define tx_pin 17

/*global variables*/
extern QueueHandle_t uart_queue;

/*function prototypes*/
esp_err_t uart_init(void);  //initialize uart communication with Renesas board

#endif