/**
 * @file state_operational.c
 * @brief Operational state implementation for the 2-gang switch
 * 
 * This file handles the operational state of the device, including:
 * - MCU communication and status queries
 * - WiFi provisioning and connectivity management
 * - MQTT cloud connection handling
 * - Periodic heartbeat message transmission
 * - Device status reporting and event handling
 */

//==============================================================================
// INCLUDES
//==============================================================================

// Custom includes
#include "network.h"
#include "state_operational.h"
#include "uart_comm.h"
#include "state_common.h"
#include "config.h"
#include "state_machine.h"

// ESP-IDF/C standard includes
#include "driver/uart.h"
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"
#include "esp_err.h"
#include "esp_heap_caps.h"


//==============================================================================
// STATIC VARIABLES
//==============================================================================

static bool wifi_initialized = false;               // WiFi initialization status flag
static TaskHandle_t heartbeatMessage_Transmission_h = NULL;  // Handle for heartbeat task
static TaskHandle_t telemetryMessage_Transmission_h = NULL;  // Handle for telemetry task
static struct state_common_data* data_lifetime;     // Pointer to persistent state data

//==============================================================================
// STATIC FUNCTION IMPLEMENTATIONS
//==============================================================================

/**
 * @brief Query product information from the MCU
 * 
 * Sends a standardized command frame to the MCU to request product information.
 * The response is expected to be handled by the UART receive callback.
 * 
 * Frame format: [Header MSB][Header LSB][Version][Command][Data Len MSB][Data Len LSB][Checksum]
 */
static void query_productInfo(void){            
    const uint8_t query_productInfo_Message[7] = {0x55  /*Frame Header MSByte*/, 
                                                  0xaa  /*Frame Header LSByte*/, 
                                                  0x00  /*Version*/, 
                                                  0x01  /*Command Word*/, 
                                                  0x00  /*Data Length MSByte*/,
                                                  0x00  /*Data Length LSByte*/, 
                                                  0x00  /*Checksum*/};    // Command to perform the query
    uint8_t bytes_sent = uart_write_bytes(uart_port, query_productInfo_Message, ( sizeof(query_productInfo_Message)/(sizeof(uint8_t)) ));
    if(bytes_sent != sizeof(query_productInfo_Message)/(sizeof(uint8_t))){
        printf("Failed to send query_productInfo_Message to MCU\n");
    }
}

/**
 * @brief Query working status from the MCU
 * 
 * Sends a command frame to the MCU to request current working status/state.
 * This includes checksum calculation for data integrity verification.
 * 
 * Command Word: 0x08 (Working Status Query)
 * Checksum: Sum of all bytes except checksum, modulo 256
 */
static void query_workingStatus(void){
    uint8_t checksum = (0x55 + 0xaa + 0x00 + 0x08 + 0x00 + 0x00) % 256;
    const uint8_t query_workingStatus_Message[7] = {0x55  /*Frame Header MSByte*/, 
                                                  0xaa  /*Frame Header LSByte*/, 
                                                  0x00  /*Version*/, 
                                                  0x08  /*Command Word*/, 
                                                  0x00  /*Data Length MSByte*/,
                                                  0x00  /*Data Length LSByte*/, 
                                                  checksum  /*Checksum*/};    // Command to perform the query
    uint8_t bytes_sent = uart_write_bytes(uart_port, query_workingStatus_Message, ( sizeof(query_workingStatus_Message)/(sizeof(uint8_t)) ));
    if(bytes_sent != sizeof(query_workingStatus_Message)/(sizeof(uint8_t))){
        printf("Failed to send query_workingStatus_Message to MCU\n");
    }
}

/**
 * @brief FreeRTOS task for periodic heartbeat message transmission
 * 
 * This task runs continuously and sends heartbeat messages to the MCU every 15 seconds
 * to maintain communication and indicate that the ESP32 is operational.
 * 
 * @param pvParameters Pointer to state_common_data structure (currently unused as we use data_lifetime)
 * 
 * @note Task runs indefinitely until suspended or deleted
 * @note Uses data_lifetime global pointer for persistent data access
 */
static void heartbeatMessage_Transmission(void *pvParameters){
    const struct state_common_data *data = pvParameters;
    while(1){
        printf("[operational] Sending heartbeat message to MCU\n");
        uart_write_bytes((uart_port_t)uart_port,data_lifetime->heartbeatMessage,sizeof(data->heartbeatMessage));
        vTaskDelay(pdMS_TO_TICKS(15000));  // 15 second delay between heartbeats
    }
}

static void telemetryMessage_Transmission(void *pvParameters){
    while(1){
        size_t heapFreeSize = heap_caps_get_free_size(MALLOC_CAP_8BIT);
        size_t telemetryMessage_size = snprintf(NULL, 0, "{\"s\": %d , \"rrsn\": %u, \"rtst\": %d, \"fh\": %d}", data_lifetime->state_machine_data->main_state, rstReason, (int)rstTimestamp, (int)heapFreeSize);
        char telemetryMessage[telemetryMessage_size + 1];
        snprintf(telemetryMessage, telemetryMessage_size + 1, "{\"s\": %d , \"rrsn\": %u, \"rtst\": %d, \"fh\": %d}", data_lifetime->state_machine_data->main_state, rstReason, (int)rstTimestamp, (int)heapFreeSize);
        esp_mqtt_client_publish(mqtt_client, telemetry_topic, telemetryMessage, strlen(telemetryMessage), 1, 0);
        vTaskDelay(pdMS_TO_TICKS(7200*1000));  // 2 hours delay between telemetry
    }
}

static void wifiStatusUpdate(uint8_t wifi_status){
    
    const uint8_t wifiStatusUpdate_Message[8] = {0x55  /*Frame Header MSByte*/, 
                                                0xaa  /*Frame Header LSByte*/, 
                                                0x00  /*Version*/, 
                                                0x03  /*Command Word*/, 
                                                0x00  /*Data Length MSByte*/, 
                                                0x01  /*Data Length LSByte*/, 
                                                wifi_status, /*Data Value*/
                                                0x00  /*Checksum*/};
    uint8_t bytes_sent = uart_write_bytes(uart_port, wifiStatusUpdate_Message, ( sizeof(wifiStatusUpdate_Message)/(sizeof(uint8_t)) ));
    if(bytes_sent != sizeof(wifiStatusUpdate_Message)/(sizeof(uint8_t))){
        printf("Failed to send wifiStatusUpdate_Message to MCU\n");
    }
}

esp_err_t update_datapoint_status(uint8_t gang_datapointID, uint8_t gang_action){
    switch(gang_datapointID){
        case 0x01:
            data_lifetime->dpStatus.dpStatus_Fields.fanPoint_1 = gang_action;
            break;
        case 0x03:
            data_lifetime->dpStatus.dpStatus_Fields.fanPoint_status = gang_action;
            break;
        case 0x65:
            data_lifetime->dpStatus.dpStatus_Fields.dataPoint_1 = gang_action;
            break;
        case 0x66:
            data_lifetime->dpStatus.dpStatus_Fields.dataPoint_2 = gang_action;
            break;
        case 0x67:
            data_lifetime->dpStatus.dpStatus_Fields.dataPoint_3 = gang_action;
            break;
        default:
            printf("[operational] Invalid gang datapoint ID: %d\n", gang_datapointID);
            return ESP_ERR_INVALID_ARG;
            break;
    }
    return ESP_OK;
}

//==============================================================================
// PUBLIC FUNCTION IMPLEMENTATIONS
//==============================================================================

/**
 * @brief Entry point for the operational state
 * 
 * Initializes the operational state by:
 * 1. Setting up persistent data pointer
 * 2. Querying MCU product information
 * 3. Waiting for product information response
 * 4. Starting heartbeat transmission task (if network config mode is 0x30)
 * 5. Initializing WiFi provisioning if not already done
 * 
 * @param data Pointer to common state data structure
 * @return esp_err_t ESP_OK on success
 */
esp_err_t operational_entry(struct state_common_data* data){
    data_lifetime = data;   // Store pointer for task access after scope ends

    printf("Operational state entered\n");

    // Query for product information from the MCU
    query_productInfo();

    // Wait for the MCU to send the product information
    xEventGroupWaitBits(events_EventGroup, EVENT_PINFO_RECEIVED_BIT, pdTRUE, pdTRUE, portMAX_DELAY);

    // Check if network configuration mode is enabled (0x30)
    if(data_lifetime->networkConfig_mode == 0x30){

        // Create or resume the heartbeat message transmission task
        if(heartbeatMessage_Transmission_h == NULL){
            BaseType_t taskCreated = xTaskCreatePinnedToCore(heartbeatMessage_Transmission,"hbMessage_Transmission", 2048,NULL, 1, &heartbeatMessage_Transmission_h, 0);
            // The TCB includes metadata for the task, including the stack pointer, task priority, and task handle
            // If the TCB is unable to get any memory allocated in the heap, it will return pdFALSE
            // This is a rare event, but it can happen if the heap is fragmented or if the module is under memory pressure
            // In this case, we restart the module to free up memory
            if(taskCreated != pdTRUE){
                printf("[operational] Failed to create heartbeat message transmission task or cloud data point status handler task. Restarting now..\n");
                esp_restart();
            }
        }
        else{
            vTaskResume(heartbeatMessage_Transmission_h);
        }

        /* Need to send a command to MCU as the module enters smart network configuration mode*/
        wifiStatusUpdate(0x00);  // 0x0 = Smart network configuration mode

        // Initialize WiFi provisioning if not already done
        if(!wifi_initialized){
            data_lifetime->cloud_status = false;
            data_lifetime->wifi_status = false;
            init_wifi_provisioning();
        }
    }
    return ESP_OK;
}

/**
 * @brief Exit point for the operational state
 * 
 * Cleans up operational state resources by suspending the heartbeat transmission task.
 * This preserves the task for potential resume rather than deleting it.
 * 
 * @param data Pointer to common state data structure (unused)
 * @return esp_err_t ESP_OK on success
 */
esp_err_t operational_exit(struct state_common_data* data){
    if(heartbeatMessage_Transmission_h != NULL){
        vTaskSuspend(heartbeatMessage_Transmission_h);  // Suspend heartbeat task
    }
    if(telemetryMessage_Transmission_h != NULL){
        vTaskSuspend(telemetryMessage_Transmission_h);  // Suspend telemetry task
    }

    ESP_ERROR_CHECK(mqtt_app_stop());
    ESP_ERROR_CHECK(esp_mqtt_client_destroy(mqtt_client));
    return ESP_OK;
}

/**
 * @brief Event handler for the operational state
 * 
 * Processes various events that can occur during operational state:
 * - MCU data point status updates
 * - Network provisioning completion
 * - WiFi connection/disconnection events
 * - Cloud connection events
 * 
 * @param event Event type identifier
 * @param data Pointer to common state data structure
 * @return esp_err_t ESP_OK on success, error code on failure
 */
esp_err_t operational_event(uint8_t event, struct state_common_data* data){
    switch(event){
        case EVENT_DP_STATUS_MCU_COMMAND:
        {
            static int seq = 0;
            static char dpStatus_message[MQTT_STATUS_BUFFER_SIZE];
            printf("[operational] Sending datapoint status to cloud\n");
            seq++;
            // Prepare the data point status message to be published to the cloud
            size_t dpStatus_size = snprintf(NULL, 0, "{\"s\": %d , \"seq\": %d}", data->dpStatus.dpStatus, seq%1000);
            snprintf(dpStatus_message, (dpStatus_size + 1), "{\"s\": %d , \"seq\": %d}", data->dpStatus.dpStatus, seq%1000);

            // Publish status to cloud if connected
            if(data_lifetime->cloud_status) esp_mqtt_client_publish(mqtt_client, status_topic, dpStatus_message, strlen(dpStatus_message), 1, 0);
            
        }
        break;

        case EVENT_NETWORK_PROVISIONED:
            // Network provisioning completed (currently no action required)
            break;
            
        case EVENT_WIFI_CONNECTED:
            // WiFi connection established
            data_lifetime->wifi_status = true;
            ESP_ERROR_CHECK(mqtt_app_start());  // Start MQTT client
            break;
            
        case EVENT_WIFI_DISCONNECTED:
            // WiFi connection lost
            if(data_lifetime->wifi_status){
                data_lifetime->wifi_status = false;
                data_lifetime->cloud_status = false;  // Cloud also disconnected
                ESP_ERROR_CHECK(mqtt_app_stop());  // Stop MQTT client
            }
            if(telemetryMessage_Transmission_h != NULL){
                vTaskSuspend(telemetryMessage_Transmission_h);  // Suspend telemetry task
            }
            wifiStatusUpdate(0x02);  // 0x02 = Disconnected from wifi yet is provisioned
            wifi_reconnect();

            break;
            
        case EVENT_NETWORK_CLOUD_CONN:
            // Cloud connection established
            data_lifetime->cloud_status = true;
            wifiStatusUpdate(0x04);  // 0x04 = Connected to cloud
            if(telemetryMessage_Transmission_h == NULL){
                BaseType_t taskCreated = xTaskCreatePinnedToCore(telemetryMessage_Transmission,"telemetryMessage_Transmission", 2048,NULL, 1, &telemetryMessage_Transmission_h, 0);
                if(taskCreated != pdTRUE){
                    printf("[operational] Failed to create telemetry message transmission task. Restarting now..\n");
                    esp_restart();
                }
            }
            else{
                vTaskResume(telemetryMessage_Transmission_h);
            }

            ESP_ERROR_CHECK(register_device(data_lifetime->dpConfig));      // Register the Z with the cloud including datapoint config
            
            query_workingStatus();  // Query current MCU status
            break;
        
        case EVENT_DP_STATUS_CLOUD_COMMAND:
            // Data point status update from cloud
            break;
            
        default:
            printf("[operational] Invalid operational event: %d\n", event);
    }
    return ESP_OK;
}