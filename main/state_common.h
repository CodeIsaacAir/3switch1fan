#ifndef STATE_COMMON_H
#define STATE_COMMON_H

#include "stdint.h"
#include "stdbool.h"
#include "freertos/FreeRTOS.h"
#include "esp_err.h"
#include "freertos/queue.h"

// Mutex for states 
extern SemaphoreHandle_t state_mutex;

/* Defines */

/* Main Events */
#define MAIN_STATE_MAX 4
#define MAIN_EVENT_MAX 16

#define EVENT_POWER_ON_BIT                              1 << 0
#define EVENT_HEARTBEAT_RECEIVED_BIT                    1 << 1
#define EVENT_PINFO_RECEIVED_BIT                        1 << 2
#define EVENT_WORKING_MODE_RECEIVED_BIT                 1 << 3
#define EVENT_MCU_RESTART_BIT                           1 << 4
#define EVENT_WIFI_RESET_COMMAND_BIT                    1 << 5          // Wi-Fi reset command
#define EVENT_NETWORK_CONFIG_COMMAND_BIT                1 << 6          // MCU selects the network config mode
#define EVENT_DP_STATUS_MCU_COMMAND_BIT                 1 << 7          // Switch(es) Control
#define EVENT_DP_STATUS_CLOUD_COMMAND_BIT               1 << 8          // Switch(es) Control
#define EVENT_NETWORK_PROVISIONED_BIT                   1 << 9          // Network Provisioned
#define EVENT_WIFI_CONNECTED_BIT                        1 << 10         // Wi-Fi Connected
#define EVENT_WIFI_DISCONNECTED_BIT                     1 << 11         // Wi-Fi Disconnected
#define EVENT_NETWORK_CLOUD_CONN_BIT                    1 << 12         // Network Cloud Connection
#define EVENT_OTA_START_BIT                             1 << 13         // OTA Start
#define EVENT_OTA_COMPLETE_BIT                          1 << 14         // OTA Complete
#define EVENT_OTA_ERROR_BIT                             1 << 15        // OTA Error
#define ALL_MAIN_EVENTS  (EVENT_POWER_ON_BIT | EVENT_HEARTBEAT_RECEIVED_BIT | EVENT_PINFO_RECEIVED_BIT | EVENT_WORKING_MODE_RECEIVED_BIT | EVENT_MCU_RESTART_BIT | EVENT_WIFI_RESET_COMMAND_BIT | EVENT_NETWORK_CONFIG_COMMAND_BIT | EVENT_DP_STATUS_MCU_COMMAND_BIT | EVENT_NETWORK_PROVISIONED_BIT | EVENT_WIFI_CONNECTED_BIT | EVENT_WIFI_DISCONNECTED_BIT | EVENT_NETWORK_CLOUD_CONN_BIT | EVENT_OTA_START_BIT | EVENT_OTA_COMPLETE_BIT | EVENT_OTA_ERROR_BIT)


/* WiFi and Cloud Connection `Events */
#define WIFI_CONNECTED_EVENT 1 << 0

/* Enums*/
typedef enum{
    STATE_POWER_ON,
    STATE_OPERATIONAL,
    STATE_FIRMWARE_UPDATE,
    STATE_ERROR
}main_state_t;

typedef enum{
    EVENT_POWER_ON,
    EVENT_HEARTBEAT_RECEIVED,
    EVENT_PINFO_RECEIVED,
    EVENT_WORKING_MODE_RECEIVED,
    EVENT_MCU_RESTART,
    EVENT_WIFI_RESET_COMMAND,
    EVENT_NETWORK_CONFIG_COMMAND,
    EVENT_DP_STATUS_MCU_COMMAND,
    EVENT_DP_STATUS_CLOUD_COMMAND,
    EVENT_NETWORK_PROVISIONED,
    EVENT_WIFI_CONNECTED,
    EVENT_WIFI_DISCONNECTED,
    EVENT_NETWORK_CLOUD_CONN,
    EVENT_OTA_START,
    EVENT_OTA_COMPLETE,
    EVENT_OTA_ERROR,
}event_t;



typedef enum{
    COPROCESSING_WORKING_MODE,
    MODULE_ONLY_MODE
}module_working_mode_t;

/* Structures and Unions */
union dpStatus_u{
    uint8_t dpStatus;
    struct{
        uint8_t dataPoint_1         :1;
        uint8_t dataPoint_2         :1;
        uint8_t dataPoint_3         :1;
        uint8_t fanPoint_1          :1;
        uint8_t fanPoint_status     :3;
    }dpStatus_Fields;
};

typedef struct {
    uint8_t from_state;
    uint8_t event;
    uint8_t to_state;
} state_transition_t;

typedef struct{
    uint8_t dataPointId_1;
    uint8_t dataPointId_2;
    uint8_t dataPointId_3;
    uint8_t fanPointId_1;
}dpConfig_t;

struct state_machine_data;
struct state_common_data{
    // Add a constant PID member when available
    //const heartbeatMessage_format heartbeatMessage;
    const uint8_t heartbeatMessage[7];
    char* mcu_version;
    struct state_machine_data *state_machine_data;  // Back pointer
    union dpStatus_u dpStatus;
    dpConfig_t dpConfig;
    uint8_t networkConfig_mode;
    uint8_t module_workingMode;
    bool wifi_status;
    bool cloud_status;
};

/* Function Pointer(s) and prototypes*/

typedef esp_err_t (*state_handler_t)(uint8_t event, struct state_common_data* data);

typedef struct{
    /* these are pointers to the functions to handle events common to all states*/
    esp_err_t (*entry)(struct state_common_data* data);
    esp_err_t (*exit)(struct state_common_data* data);
    esp_err_t (*event)(uint8_t event, struct state_common_data* data);
    state_handler_t event_handlers[MAIN_EVENT_MAX];
}state_t;

esp_err_t publishStatusToMCU(uint8_t gang_datapointID, uint8_t gang_action);
/* Global Variables */
extern EventGroupHandle_t events_EventGroup;
extern QueueHandle_t mqttData_queue;


#endif