/*
 * ============================================================================
 * STATE MACHINE IMPLEMENTATION
 * ============================================================================
 * This file implements the main state machine for the 2-gang switch project.
 * It handles state transitions, UART communication, and event processing.
 * 
 * The state machine manages two primary states:
 * - STATE_POWER_ON: Initial state after power-up
 * - STATE_OPERATIONAL: Normal operation state
 * 
 * Events are processed through an event group mechanism and handled by
 * state-specific event handlers.
 * ============================================================================
 */

/* ============================================================================
 * INCLUDES
 * ============================================================================ */

/* Standard C Library Includes */
#include "stdbool.h"
#include "assert.h"
#include "string.h"
#include "stdint.h"
#include "time.h"

/* FreeRTOS Includes */
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"

/* ESP-IDF Driver Includes */
#include "driver/uart.h"
#include "esp_err.h"
#include "config.h"
#include "esp_system.h"
#include "nvs_flash.h"

/* Project-Specific Includes */
#include "uart_comm.h"
#include "state_machine.h"
#include "state_common.h"
#include "state_poweron.h"
#include "state_operational.h"
#include "state_firmware_update.h"

/* ============================================================================
 * CONSTANTS AND GLOBAL VARIABLES
 * ============================================================================ */

/* NVS handle to store device configuration */
static nvs_handle_t device_config_h;

/* Global state machine data structure with pre-initialized heartbeat message */
static struct state_machine_data state_machine_data = {
    .common.heartbeatMessage = {
        0x55,  /* Frame Header [MSByte] */
        0xAA,  /* Frame Header [LSByte] */
        0x00,  /* Version */
        0x00,  /* Data Length[MSByte] */
        0x00,  /* Data Length[LSByte] */
        0x00,  /* Data */
        0xff   /* Checksum */
    }
};

/* Reset reason and timestamp */
esp_reset_reason_t rstReason;
time_t rstTimestamp = 0;

/* Synchronization primitives for thread-safe state management */
SemaphoreHandle_t state_mutex = NULL;           /* Mutex for state data protection */
EventGroupHandle_t events_EventGroup = NULL;   /* Event group for inter-task communication */
QueueHandle_t mqttData_queue = NULL;            /* Queue for MQTT data */

/* ============================================================================
 * FUTURE ENHANCEMENTS
 * ============================================================================ */

/* @TODO: Need to implement this to validate state transition
 * This table will be used to enforce valid state transitions and prevent
 * invalid state changes that could lead to system instability.
 */
/* 
static const struct state_transition state_transition_table[] = {
    {STATE_POWER_ON, EVENT_POWER_ON, STATE_HEARTBEAT_RESPONSE_WAIT},
    {STATE_HEARTBEAT_RESPONSE_WAIT, EVENT_NO_HEARTBEAT_RECEIVED, STATE_HEARTBEAT_RESPONSE_WAIT},
    {STATE_HEARTBEAT_RESPONSE_WAIT, EVENT_HEARTBEAT_RECEIVED, STATE_OPERATIONAL},
};
*/

/* ============================================================================
 * FUNCTION DECLARATIONS
 * ============================================================================ */

/* Event handler function prototypes */
static esp_err_t powerOn_entry_handler(uint8_t event, struct state_common_data *data);
static esp_err_t hbRecv_handler(uint8_t event, struct state_common_data *data);
static esp_err_t mcuInfo_handler(uint8_t event, struct state_common_data *data);
static esp_err_t operationalEvent_handler(uint8_t event, struct state_common_data *data);
static esp_err_t otaEntry_handler(uint8_t event, struct state_common_data *data);
static esp_err_t otaEvent_handler(uint8_t event, struct state_common_data *data);

/* ============================================================================
 * STATE TABLE DEFINITION
 * ============================================================================ */


 // Add this to state_machine.c
static const state_transition_t valid_transitions[] = {
    // Power-On State Transitions
    {STATE_POWER_ON, EVENT_POWER_ON, STATE_POWER_ON},                    // Stay in power-on
    {STATE_POWER_ON, EVENT_HEARTBEAT_RECEIVED, STATE_OPERATIONAL},       // Normal transition
    
    // Operational State Transitions  
    {STATE_OPERATIONAL, EVENT_OTA_START, STATE_FIRMWARE_UPDATE},         // Start OTA
    {STATE_OPERATIONAL, EVENT_MCU_RESTART, STATE_POWER_ON},              // Restart scenario
    
    // Firmware Update State Transitions
    {STATE_FIRMWARE_UPDATE, EVENT_OTA_COMPLETE, STATE_FIRMWARE_UPDATE},  // System restart
    {STATE_FIRMWARE_UPDATE, EVENT_OTA_ERROR, STATE_OPERATIONAL},         // Error recovery
    
    // End marker
    {MAIN_STATE_MAX, 0, 0}
};

/**
 * @brief State table containing state handlers and event mappings
 * 
 * This table defines the behavior for each state in the system, including:
 * - Entry functions: Called when entering a state
 * - Exit functions: Called when leaving a state  
 * - Event handlers: Functions to handle specific events in each state
 * - Event processors: General event processing functions for states
 */
static state_t state_table[MAIN_STATE_MAX] = {
    /* Power-On State: Initial state after system startup */
    [STATE_POWER_ON] = {
        .entry = powerOn_entry,
        .exit = powerOn_exit,
        .event_handlers = {
            [EVENT_POWER_ON] = powerOn_entry_handler,           /* Handle power-on event */
            [EVENT_HEARTBEAT_RECEIVED] = hbRecv_handler,        /* Handle heartbeat reception */
        }
    },
    
    /* Operational State: Normal operation mode */
    [STATE_OPERATIONAL] = {
        .entry = operational_entry,
        .exit = operational_exit,
        .event = operational_event,
        .event_handlers = {
            [EVENT_PINFO_RECEIVED] = mcuInfo_handler,           /* Product information received */
            [EVENT_WORKING_MODE_RECEIVED] = mcuInfo_handler,    /* Working mode information */
            [EVENT_DP_STATUS_MCU_COMMAND] = operationalEvent_handler,  /* Data point status updates */
            [EVENT_NETWORK_PROVISIONED] = operationalEvent_handler,    /* Network provisioning events */
            [EVENT_WIFI_CONNECTED] = operationalEvent_handler,         /* WiFi connection events */
            [EVENT_WIFI_DISCONNECTED] = operationalEvent_handler,      /* WiFi disconnection events */
            [EVENT_NETWORK_CLOUD_CONN] = operationalEvent_handler,     /* Cloud connection events */
            [EVENT_DP_STATUS_CLOUD_COMMAND] = operationalEvent_handler, /* Data point status updates */
            [EVENT_OTA_START] = otaEntry_handler,               /* OTA start trigger */
        }
    },

    /* Firmware Update State: OTA firmware update mode */
    [STATE_FIRMWARE_UPDATE] = {
        .entry = firmware_update_entry,
        .exit = firmware_update_exit,
        .event = firmware_update_event,
        .event_handlers = {
            [EVENT_OTA_COMPLETE] = otaEvent_handler,                   /* OTA completion */
            [EVENT_OTA_ERROR] = otaEvent_handler,                      /* OTA error handling */
        }
    },

};

/* ============================================================================
 * UTILITY FUNCTIONS
 * ============================================================================ */

/**
 * @brief Convert event bit mask to event enumeration
 * @param event Event bit mask from event group
 * @return Event enumeration value (0-based index of first set bit)
 * 
 * This function finds the first set bit in the event mask and returns
 * its position as an enumeration value for use with the state table.
 */
static inline uint8_t bit_to_enum(uint16_t event) {
    int i;
    for (i = 0; i < MAIN_EVENT_MAX; i++) {
        if (event & (1 << i)) {
            break;
        }
    }
    return i;
}

/* ============================================================================
 * MAIN STATE MACHINE TASKS
 * ============================================================================ */

/**
 * @brief Main state transition checking task
 * @param pvParamaters Unused task parameter
 * 
 * This task continuously waits for events from the event group and dispatches
 * them to the appropriate state handlers. It runs indefinitely and forms the
 * core of the state machine event processing loop.
 * 
 * Event Flow:
 * 1. Wait for any event bit to be set in the event group
 * 2. Convert event bits to enumeration
 * 3. Look up appropriate handler in current state's event handler table
 * 4. Execute handler if found, log error if not found
 * 5. Repeat
 */
static void stateTransitionCheck(void *pvParamaters) {
    while (1) {
        /* Wait for any event, clear bits after reading, don't wait for all bits */
        EventBits_t events = xEventGroupWaitBits(events_EventGroup, ALL_MAIN_EVENTS, pdTRUE, pdFALSE, portMAX_DELAY);
        /* Convert bit mask to event enumeration */
        uint8_t event = bit_to_enum(events);
        /* Get the handler for the event from the current state's event table */
        state_handler_t handler = state_table[state_machine_data.main_state].event_handlers[event];
        if (handler) {
            /* Execute the event handler */
            handler(event, &state_machine_data.common);
        } else {
            printf("[statemachine] No handler found for event %d\n", event);
        }
    }
}


/**
 * @brief Parse and process incoming UART commands
 * @param input_data Pointer to received data buffer
 * @param buffer_size Size of the received data
 * 
 * This function parses incoming UART messages and processes them according
 * to the communication protocol. It extracts the command word and handles
 * different types of messages from the MCU.
 * 
 * Supported Commands:
 * - 0x00: Heartbeat Detection Response
 * - 0x01: Query Product Information Response  
 * - 0x02: Query Working Mode Response
 * - 0x03: Wi-Fi Status Response
 * - 0x07: Data Point Status Updates
 */
static esp_err_t commandCheck(const uint8_t *input_data, uint8_t buffer_size) {
    /* Extract command word from protocol frame (byte 3) */
    /* need to add buffer size check */
    uint8_t commandWord = input_data[3];
    printf("The command word is 0x%02x\n", commandWord);
    
    switch (commandWord) {
        /* Heartbeat Detection Response (0x00) */
        case 0x00:
        {
            uint8_t data = input_data[6];
            if (data == 0x00) {
                xEventGroupSetBits(events_EventGroup, EVENT_HEARTBEAT_RECEIVED_BIT);
            } else if (data == 0x01) {
                printf("System Functioning Well. State %d\n", state_machine_data.main_state);
            }
        }
        break;

        /* Query Product Information Response (0x01) */
        case 0x01:
            if (xSemaphoreTake(state_mutex, portMAX_DELAY) == pdTRUE) {
                /* Extract network configuration mode from byte 46 */
                state_machine_data.common.networkConfig_mode = input_data[46];
                xEventGroupSetBits(events_EventGroup, EVENT_PINFO_RECEIVED_BIT);
                xSemaphoreGive(state_mutex);
            }
            break;

        /* Query Working Mode of the Module (0x02) */
        case 0x02:
            if (buffer_size == 7) {
                xEventGroupSetBits(events_EventGroup, EVENT_WORKING_MODE_RECEIVED_BIT);  
            }
            break;

        /* Wi-Fi Status Response from MCU (0x03) */
        case 0x03:
            /* Check the documentation to see how the on-panel indicator 
             * would respond to various network sub-states */
            printf("[statemachine] Check the panel indicator for the network status\n");
            break;

        /* Data Point Status Updates (0x07) */
        case 0x07: {
            if (xSemaphoreTake(state_mutex, portMAX_DELAY) == pdTRUE) {
                /* Extract data point ID and status from protocol frame */
                uint8_t dp_id = input_data[6];      /* Data point identifier */
                uint8_t dp_status = input_data[10]; /* Data point status value */
                
                /* Update corresponding data point based on ID */
                switch (dp_id) {
                    case 0x01:
                        state_machine_data.common.dpStatus.dpStatus_Fields.fanPoint_1 = dp_status ? 1 : 0;
                        break;
                    case 0x03:
                        for(int i = 0; i< buffer_size; i++){
                            printf("[statemachine] 0x%02x at %d\n", input_data[i], i);
                        }
                        state_machine_data.common.dpStatus.dpStatus_Fields.fanPoint_status = dp_status;
                        break;
                    case 0x65:
                        state_machine_data.common.dpStatus.dpStatus_Fields.dataPoint_1 = dp_status ? 1 : 0;
                        break;
                    case 0x66:
                        state_machine_data.common.dpStatus.dpStatus_Fields.dataPoint_2 = dp_status ? 1 : 0;
                        break;
                    case 0x67:
                        state_machine_data.common.dpStatus.dpStatus_Fields.dataPoint_3 = dp_status ? 1 : 0;
                        break;
                    default:
                        printf("[statemachine] Invalid data point ID\n");
                        break;
                }
                
                /* Signal that data point status has been updated */
                xEventGroupSetBits(events_EventGroup, EVENT_DP_STATUS_MCU_COMMAND_BIT);
                xSemaphoreGive(state_mutex);
            } else {
                printf("[statemachine] Failed to take state mutex\n");
            }
            break;
        }

        /* Unknown/Invalid Commands */
        default:
            printf("[statemachine] Invalid command received: 0x%02x\n", commandWord);
            /* Log the entire buffer for debugging purposes */
            for (int i = 0; i < buffer_size; i++) {
                printf("[statemachine] 0x%02x at %d\n", input_data[i], i);
            }

            return ESP_ERR_INVALID_ARG;
    }
    return ESP_OK;
}

/**
 * @brief UART event processing task
 * @param pvParameters Unused task parameter
 * 
 * This task handles all UART events from the hardware queue. It processes
 * incoming data, error conditions, and other UART-related events.
 * 
 * Event Types Handled:
 * - UART_DATA: Incoming data available for processing
 * - UART_BUFFER_FULL: UART buffer overflow condition
 * - UART_FIFO_OVF: UART FIFO overflow
 * - UART_BREAK: Break condition detected
 * - UART_PARITY_ERR: Parity error in received data
 * - UART_FRAME_ERR: Frame error in received data
 * - UART_PATTERN_DET: Pattern detection event
 * - UART_DATA_BREAK: Data break condition
 */
static void uart_event_task(void *pvParameters) {
    uart_event_t event;
    size_t buffered_size;

    while (1) {
        /* Wait for UART events from the hardware queue */
        assert(uart_queue);
        if (xQueueReceive(uart_queue, &event, portMAX_DELAY)) {
            switch (event.type) {
                /* Data received and ready for processing */
                case UART_DATA:
                    /* Get the amount of data available in the ring buffer */
                    uart_get_buffered_data_len(uart_port, &buffered_size);
                    if (buffered_size > 0 && buffered_size <= UART_BUFFER_SIZE) {
                        
                        /* Allocate buffer for incoming data */
                        uint8_t rx_buffer[buffered_size];
                        
                        /* Read data from UART with timeout */
                        uart_read_bytes(uart_port, rx_buffer, buffered_size, pdMS_TO_TICKS(200));
                            
                        /* Process the received command */
                        esp_err_t ret = commandCheck(rx_buffer, (uint8_t)buffered_size);
                        if(ret != ESP_OK){
                            printf("[statemachine] Failed to process command\n");
                        }
                        /* Clear any remaining data in the buffer */
                        uart_flush(uart_port);
                    }
                    break;

                /* UART buffer overflow - data may be lost */
                case UART_BUFFER_FULL:
                    printf("[statemachine] UART buffer full\n");
                    break;

                /* UART FIFO overflow - hardware level overflow */
                case UART_FIFO_OVF:
                    printf("[statemachine] UART FIFO overflow\n");
                    uart_flush(uart_port);
                    uart_flush_input(uart_port);
                    break;

                /* Break condition detected (extended low signal) */
                case UART_BREAK:
                    printf("[statemachine] UART break. Null character received\n");
                    break;

                /* Parity error in received data */
                case UART_PARITY_ERR:
                    printf("[statemachine] UART parity error\n");
                    break;

                /* Frame error in received data */
                case UART_FRAME_ERR:
                    printf("[statemachine] UART frame error\n");
                    break;

                /* Pattern detection event (if configured) */
                case UART_PATTERN_DET:
                    break;

                /* Data break condition */
                case UART_DATA_BREAK:
                    break;

                /* Invalid event type */
                case UART_EVENT_MAX:
                    printf("[statemachine] UART event max\n");
                    break;
            }
        }
    }
}


/* ============================================================================
 * INITIALIZATION AND UTILITY FUNCTIONS
 * ============================================================================ */

/**
 * @brief Load/Store datapoint configuration from/to NVS
 * @return ESP_OK on success, error code otherwise
 * 
 * Handles NVS initialization with automatic erase if needed
 */

 static esp_err_t dataPointConfig_nvs(void){
    esp_err_t ret = nvs_open("device_config", NVS_READWRITE, &device_config_h);
    if (ret != ESP_OK) {
        printf("[statemachine] Failed to open NVS partition\n");
        return ret;
    }

    esp_err_t dp1Ret = nvs_get_u8(device_config_h, "dataPointId_1", &state_machine_data.common.dpConfig.dataPointId_1);
    esp_err_t dp2Ret = nvs_get_u8(device_config_h, "dataPointId_2", &state_machine_data.common.dpConfig.dataPointId_2);
    esp_err_t dp3Ret = nvs_get_u8(device_config_h, "dataPointId_3", &state_machine_data.common.dpConfig.dataPointId_3);
    esp_err_t dp4Ret = nvs_get_u8(device_config_h, "dataPointId_4", &state_machine_data.common.dpConfig.fanPointId_1);

    if(dp1Ret == ESP_ERR_NVS_NOT_FOUND || dp2Ret == ESP_ERR_NVS_NOT_FOUND || dp3Ret == ESP_ERR_NVS_NOT_FOUND || dp4Ret == ESP_ERR_NVS_NOT_FOUND){
        nvs_set_u8(device_config_h, "dataPointId_1", 0x65);
        nvs_set_u8(device_config_h, "dataPointId_2", 0x66);
        nvs_set_u8(device_config_h, "dataPointId_3", 0x67);
        nvs_set_u8(device_config_h, "dataPointId_4", 0x01);
    }

    nvs_close(device_config_h); // Close NVS handle
    return ESP_OK;
}


/**
 * @brief Initialize state machine data structure
 * @param state_machine_init_data Pointer to state machine data to initialize
 * 
 * This function sets up the initial state and default values for the state
 * machine data structure. It configures the initial state, sets up back
 * pointers, and initializes default working modes.
 */
static inline void state_machine_init(struct state_machine_data *state_machine_init_data) {
    /* Set initial state to power-on */
    state_machine_init_data->main_state = STATE_POWER_ON;
    
    /* Set up back pointer for common access to state machine data */
    state_machine_init_data->common.state_machine_data = state_machine_init_data;
    
    /* Initialize data point status to all off */
    state_machine_init_data->common.dpStatus.dpStatus = 0x00;
    
    /* Set default working mode to co-processing */
    state_machine_init_data->common.module_workingMode = COPROCESSING_WORKING_MODE;

    // Retrieve data point configuration from NVS
    dataPointConfig_nvs();
}

/**
 * @brief Start the state machine execution
 * 
 * This is the main entry point for the state machine. It initializes all
 * synchronization primitives, sets up the state machine data, and creates
 * the required tasks for state transition management and UART event handling.
 * 
 * Tasks Created:
 * - stateTransitionCheck: Handles state transitions based on events
 * - uart_event_task: Processes UART communication events
 */
void state_machine_run(void) {
    /* Create synchronization primitives for thread-safe operation */
    events_EventGroup = xEventGroupCreate();
    mqttData_queue = xQueueCreate(10, sizeof(char*));
    state_mutex = xSemaphoreCreateMutex();

    if (events_EventGroup != NULL) {
        /* Initialize the global state machine data structure */
        state_machine_init(&state_machine_data);
        
        /* Trigger the initial power-on event */
        if(esp_reset_reason() == ESP_RST_POWERON){
            rstReason = ESP_RST_POWERON;
            rstTimestamp = time(NULL);
            printf("[statemachine] Power on reset detected\n");
            xEventGroupSetBits(events_EventGroup, EVENT_POWER_ON_BIT);
        } else {
            rstReason = esp_reset_reason();
            rstTimestamp = time(NULL);
            printf("[statemachine] Reset detected\n");
            xEventGroupSetBits(events_EventGroup, EVENT_HEARTBEAT_RECEIVED_BIT);
        }
        
        /* Create core state machine tasks */
        /* State transition task - handles event-driven state changes */
        xTaskCreatePinnedToCore(stateTransitionCheck, "stateTransitionCheck", 4096, NULL, 10, NULL, 0);
        
        /* UART event task - handles communication with MCU */
        xTaskCreatePinnedToCore(uart_event_task, "capture_uart_events", 4096, NULL, 10, NULL, 0);
    }
}

/**
 * @brief Validate if a state transition is legal
 * @param from Current state
 * @param event Triggering event
 * @param to Target state
 * @return true if transition is valid, false otherwise
 */
static bool is_valid_transition(uint8_t from, uint8_t event, uint8_t to) {
    for (int i = 0; valid_transitions[i].from_state != MAIN_STATE_MAX; i++) {
        if (valid_transitions[i].from_state == from &&
            valid_transitions[i].event == event &&
            valid_transitions[i].to_state == to) {
            return true;
        }
    }
    return false;
}


/**
 * @brief Safely change state from one state to another with validation
 * @param from Current state to exit from
 * @param to Target state to enter
 * @param data Pointer to common state data
 * 
 * This function performs a thread-safe state transition by:
 * 1. Calling the exit handler for the current state
 * 2. Updating the state variable under mutex protection
 * 3. Calling the entry handler for the new state
 */
static inline void change_state(uint8_t from, uint8_t to, struct state_common_data *data, uint8_t event) {
    // Validate the state transition
    if (!is_valid_transition(from, event, to)) {
        printf("[statemachine] State transition rejected: %d -> %d\n", from, to);
        return;
    }

    /* Call exit handler for current state if it exists */
    if (state_table[from].exit) {
        state_table[from].exit(data);
    }
    /* Update state under mutex protection */
    if (xSemaphoreTake(state_mutex, portMAX_DELAY) == pdTRUE) {
        /* Set the new state safely once the mutex is acquired */
        data->state_machine_data->main_state = to;
        xSemaphoreGive(state_mutex);
    } else {
        printf("[statemachine] Failed to take state mutex\n");
    }

    /* Call entry handler for new state if it exists */
    if (state_table[to].entry) {
        state_table[to].entry(data);
    }
    else {
        printf("[statemachine] No entry handler found for the %d state\n", to);
    }
}

/* ============================================================================
 * EVENT HANDLER IMPLEMENTATIONS
 * ============================================================================ */

/**
 * @brief Handle power-on entry events
 * @param event Event type that triggered this handler
 * @param data Pointer to common state data
 * @return ESP_OK on success
 * 
 * This handler is called when the system transitions into the power-on state.
 * It triggers the power-on state's entry function to perform initialization.
 */
esp_err_t powerOn_entry_handler(uint8_t event, struct state_common_data *data) {
    ESP_ERROR_CHECK(state_table[STATE_POWER_ON].entry(&data->state_machine_data->common));
    return ESP_OK;
}

/**
 * @brief Handle heartbeat reception events
 * @param event Event type that triggered this handler  
 * @param data Pointer to common state data
 * @return ESP_OK on success
 * 
 * This handler processes heartbeat messages from the MCU. Upon receiving
 * the initial heartbeat, it transitions the system from power-on state
 * to operational state, indicating successful communication establishment.
 */
esp_err_t hbRecv_handler(uint8_t event, struct state_common_data *data) {
    change_state(STATE_POWER_ON, STATE_OPERATIONAL, &data->state_machine_data->common, event);
    return ESP_OK;
}

/**
 * @brief Handle operational state events
 * @param event Event type that triggered this handler
 * @param data Pointer to common state data  
 * @return ESP_OK on success
 * 
 * This handler processes events that occur during normal operational state.
 * It delegates event processing to the operational state's event processor
 * which handles network, WiFi, and cloud connectivity events.
 */
esp_err_t operationalEvent_handler(uint8_t event, struct state_common_data *data) {
    state_table[STATE_OPERATIONAL].event(event, &data->state_machine_data->common);
    return ESP_OK;
}

/**
 * @brief Handle MCU information events
 * @param event Event type that triggered this handler
 * @param data Pointer to common state data
 * @return ESP_OK on success
 * 
 * This handler processes MCU information messages such as product information
 * and working mode responses. Currently logs the reception but can be extended
 * to perform additional processing of MCU configuration data.
 */
esp_err_t mcuInfo_handler(uint8_t event, struct state_common_data *data) {
    printf("MCU information received\n");
    return ESP_OK;
}

/**
 * @brief Handle OTA-related events
 * @param event Event type that triggered this handler
 * @param data Pointer to common state data
 * @return ESP_OK on success
 * 
 * This handler processes OTA events such as OTA start triggers, completion,
 * and error conditions. It manages state transitions between operational
 * and firmware update states.
 */
esp_err_t otaEntry_handler(uint8_t event, struct state_common_data *data) {
    change_state(STATE_OPERATIONAL, STATE_FIRMWARE_UPDATE, data, event);
    return ESP_OK;
}

/**
 * @brief Handle OTA-related events
 * @param event Event type that triggered this handler
 * @param data Pointer to common state data
 * @return ESP_OK on success
 * 
 * This handler processes OTA events such as OTA start triggers, completion,
 * and error conditions. It manages state transitions between operational
 * and firmware update states.
 */
esp_err_t otaEvent_handler(uint8_t event, struct state_common_data *data) {
    switch(event) {
        case EVENT_OTA_COMPLETE:
            state_table[STATE_FIRMWARE_UPDATE].exit(data);
            state_table[STATE_FIRMWARE_UPDATE].event(event, data);
            break;
            
        case EVENT_OTA_ERROR:
            printf("[statemachine] OTA failed, returning to operational state\n");
            //change_state(STATE_FIRMWARE_UPDATE, STATE_OPERATIONAL, data);
            break;
            
        default:
            printf("[statemachine] Unhandled OTA event: %d\n", event);
            break;
    }
    return ESP_OK;
}