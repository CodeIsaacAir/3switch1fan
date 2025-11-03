/**
 * @file network.c
 * @brief Network management module for ESP32 smart switch
 * 
 * This module handles:
 * - WiFi provisioning via BLE
 * - MQTT client connectivity and messaging
 * -  s management in NVS
 * - WiFi reconnection with backoff

 * 
 * @author Smart Switch Team
 * @date 2024
 */

/*==============================================================================
 * INCLUDES
 *============================================================================*/

// Custom includes
#include "network.h"
#include "config.h"
#include "state_common.h"

// Standard C includes
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <assert.h>
#include "tiny-json.h"

// Standard ESP-IDF includes
#include "esp_err.h"
#include "nvs_flash.h"
#include "mqtt_client.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_app_desc.h"

// WiFi provisioning includes
#include "wifi_provisioning/manager.h"
#include "wifi_provisioning/scheme_ble.h"

/*==============================================================================
 * DEFINES AND CONSTANTS
 *============================================================================*/

/*==============================================================================
 * GLOBAL VARIABLES
 *============================================================================*/
esp_mqtt_client_handle_t mqtt_client = NULL;        // MQTT client handle
static wifi_prov_security_t security = WIFI_PROV_SECURITY_0;  // Provisioning security
static esp_err_t status = WIFI_FAILURE;             // WiFi connection status
static char *user_id = NULL;                        // User ID buffer
static nvs_handle_t device_config_h;                       // NVS handle for device config
char firmware_url[512] = {0};                        // Firmware URL

// MQTT topic strings - dynamically allocated based on user_id and device_id
char* lwt_topic = NULL;                              // Last Will and Testament topic
char* status_topic = NULL;                          // Device status topic
char* register_topic = NULL;                        // Device registration topic
char* command_topic = NULL;                        // Device command topic
char* ota_topic = NULL;                            // OTA update topic
char* telemetry_topic = NULL;                      // Telemetry topic

/*==============================================================================
 * REGISTER DEVICE FUNCTION
 *============================================================================*/

/**
 * @brief Register device with the cloud
 * @param dpConfig Data point configuration structure containing datapoint IDs
 * @return ESP_OK on success, error code otherwise
 * 
 * Registers the device with the cloud using the MQTT client, including
 * datapoint configuration for finer control by the cloud service.
 */
esp_err_t register_device(dpConfig_t dpConfig){

    assert(user_id);
    assert(mqtt_client);

    // Create JSON registration message with datapoint configuration
    char registrationMessage[352];
    const esp_app_desc_t *running_app_info = esp_app_get_description();
    size_t registrationMessage_size = snprintf(NULL, 0, 
        "{\"dev_id\":\"%s\",\"u_id\":\"%s\",\"type\":\"%s\",\"ver\":\"%s\",\"dpConfig\":[{\"dp1_id\":%d, \"sup_types\":[\"light\"]},{\"dp2_id\":%d, \"sup_types\":[\"light\"]},{\"dp3_id\":%d, \"sup_types\":[\"light\"]},{\"dp4_id\":%d, \"sup_types\":[\"fan\"]}]}", 
        device_id, user_id, device_type, running_app_info->version, dpConfig.dataPointId_1, dpConfig.dataPointId_2, dpConfig.dataPointId_3, dpConfig.fanPointId_1);
    snprintf(registrationMessage, (registrationMessage_size + 1), 
        "{\"dev_id\":\"%s\",\"u_id\":\"%s\",\"type\":\"%s\",\"ver\":\"%s\",\"dpConfig\":[{\"dp1_id\":%d, \"sup_types\":[\"light\"]},{\"dp2_id\":%d, \"sup_types\":[\"light\"]},{\"dp3_id\":%d, \"sup_types\":[\"light\"]},{\"dp4_id\":%d, \"sup_types\":[\"fan\"]}]}", 
        device_id, user_id, device_type, running_app_info->version, dpConfig.dataPointId_1, dpConfig.dataPointId_2, dpConfig.dataPointId_3, dpConfig.fanPointId_1);

    esp_mqtt_client_publish(mqtt_client, register_topic, registrationMessage, strlen(registrationMessage), 1, 0);

    return ESP_OK;
}

/*==============================================================================
 * NVS MANAGEMENT FUNCTIONS
 *============================================================================*/

/**
 * @brief Retrieve user ID from NVS storage
 * @param user_id Pointer to store the retrieved user ID
 * @note Allocates memory for user_id that should be freed by caller
 */
void retrieve_user_id(){
    // Retrieve the user ID from NVS and safely close the storage handle
    size_t required_size;

    if(device_config_h){
        printf("[wifi] Device config NVS already opened\n");
        nvs_get_str(device_config_h, "user_id", NULL, &required_size);
        user_id = malloc(required_size);  // allocate memory in heap for user_id
        nvs_get_str(device_config_h, "user_id", user_id, &required_size);  // get the user_id from NVS and store it in user_id
        printf("[wifi] User ID: %s\n", user_id);
        nvs_close(device_config_h);
    }
    else{
        printf("[wifi] Opening device config NVS\n");
        nvs_open("device_config", NVS_READONLY, &device_config_h);
        nvs_get_str(device_config_h, "user_id", NULL, &required_size);
        user_id = malloc(required_size);  // allocate memory in heap for user_id
        nvs_get_str(device_config_h, "user_id", user_id, &required_size);  // get the user_id from NVS and store it in user_id
        if(user_id == NULL){
            printf("[wifi] User ID not found in NVS\n");
            return;
        }
        nvs_close(device_config_h);
    }
}

/*==============================================================================
 * MQTT FUNCTIONS
 *============================================================================*/

/**
 * @brief MQTT subscription handler
 * @param void
 * 
 * Subscribes to all relevant MQTT topics for this device.
 * Uses pre-allocated topic strings that were created in mqtt_app_start().
 * Topics include:
 * - status: Device status updates and commands
 * - telemetry: Environmental data and sensor readings
 * - register: Device registration and configuration messages
 * 
 * Note: LWT topic is not subscribed to as it's handled automatically by broker
 */
static inline void mqtt_subscription_handler(){
    
    // Subscribe to status topic for device status updates
    esp_mqtt_client_subscribe(mqtt_client, command_topic, 0);

    // Subscribe to registration topic for device management
    esp_mqtt_client_subscribe(mqtt_client, register_topic, 0);

    // Subscribe to OTA topic for OTA updates
    esp_mqtt_client_subscribe(mqtt_client, ota_topic, 0);
}

/**
 * @brief MQTT event handler
 * @param handler_args Event handler arguments
 * @param base Event base
 * @param event_id Event ID
 * @param event_data Event data
 * 
 * Handles MQTT connection, disconnection, errors, and publish confirmations
 */
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = event_data;
    
    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
            printf("MQTT Connected to broker\n");
            mqtt_subscription_handler();
            xEventGroupSetBits(events_EventGroup, EVENT_NETWORK_CLOUD_CONN_BIT);
            break;
            
        case MQTT_EVENT_DISCONNECTED:
            printf("[wifi] MQTT Disconnected from broker\n");
            break;
            
        case MQTT_EVENT_ERROR:
            printf("[wifi] MQTT Error occurred\n");
            if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
                printf("[wifi] Last error code reported from esp-tls: 0x%x\n", event->error_handle->esp_tls_last_esp_err);
                printf("[wifi] Last tls stack error number: 0x%x\n", event->error_handle->esp_tls_stack_err);
                printf("[wifi] Last captured errno : %d (%s)\n", event->error_handle->esp_transport_sock_errno,
                        strerror(event->error_handle->esp_transport_sock_errno));
            }
            break;
            
        case MQTT_EVENT_PUBLISHED:
            //printf("Message successfully published, msg_id=%d\n", event->msg_id);
            break;

        case MQTT_EVENT_DATA:
        {
            //printf("[wifi] MQTT data received\n");
            static char mqtt_topic[128];
            memcpy(mqtt_topic, event->topic, event->topic_len);
            mqtt_topic[event->topic_len] = '\0';
            
            if(strcmp(mqtt_topic, command_topic) == 0){  // Command topic
                if(event->data_len > MQTT_COMMAND_BUFFER_SIZE){
                    //printf("[wifi] MQTT data length exceeds maximum allowed\n");
                    break;
                }
                char mqtt_data_copy[MQTT_COMMAND_BUFFER_SIZE]; // MQTT data copy buffer
                memcpy(mqtt_data_copy, event->data, event->data_len);
                mqtt_data_copy[event->data_len] = '\0'; // Ensure null termination

                printf("[wifi] MQTT data: %s\n", mqtt_data_copy);
                
                // Parse JSON to determine command type
                json_t pool[10]; //type, dp_id, action
                json_t const *root = json_create(mqtt_data_copy, pool, 10);
                if (root) {
                    json_t const *commandType = json_getProperty(root, "type"); // Property lookup
                    if (commandType) {
                        if (strcmp(json_getValue(commandType), "gang_control") == 0) {  // Getting the value
                            // Forward gang control commands to the operational state handler
                            json_t const *dp_id = json_getProperty(root, "dp_id");
                            json_t const *action = json_getProperty(root, "action");
                            if(dp_id && action){
                                esp_err_t ret = publishStatusToMCU((uint8_t)json_getInteger(dp_id), (uint8_t)json_getInteger(action));
                                if(ret == ESP_OK){
                                    xEventGroupSetBits(events_EventGroup, EVENT_DP_STATUS_MCU_COMMAND_BIT);
                                }
                                else{
                                    printf("[wifi] Failed to publish status to MCU\n");
                                }
                            }else{
                                printf("[wifi] Missing or invalid dp_id or action in MQTT message\n");
                            }
                        }else {
                            printf("[wifi] Unknown command: %s\n", json_getValue(commandType));
                        }
                    }
                }else {
                    printf("[wifi] json_create failed (bad JSON or pool too small)\n");
                }
            }else if(strcmp(mqtt_topic, ota_topic) == 0){  // OTA update topic
                char *mqtt_data_copy = malloc(event->data_len + 1);
                if(!mqtt_data_copy){    
                    printf("[wifi] Failed to allocate memory for mqtt_data_copy\n");
                    break;
                }
                memcpy(mqtt_data_copy, event->data, event->data_len);
                mqtt_data_copy[event->data_len] = '\0'; // Ensure null termination
                
                json_t pool[10];
                json_t const *root = json_create(mqtt_data_copy, pool, 10);
                if (root) {
                    json_t const *commandType = json_getProperty(root, "type"); // Property lookup
                    json_t const *url = json_getProperty(root, "url");
                    if (commandType && url) {
                        if (strcmp(json_getValue(commandType), "ota_update") == 0) {
                            const char *url_value = json_getValue(url);

                            if(url_value){
                                if(strlen(url_value) > (sizeof(firmware_url) - 1)){
                                    //printf("[wifi] URL length exceeds maximum allowed\n");
                                    break;
                                }
                                snprintf(firmware_url, (strlen(url_value) + 1), "%s", url_value);
                                //printf("[wifi] OTA URL: %s\n", firmware_url);
                                xEventGroupSetBits(events_EventGroup, EVENT_OTA_START_BIT);
                            }
                        }
                    }
                    else {
                        printf("[wifi] Missing or invalid command_type or url in MQTT message\n");
                    }
                }
                else {
                    printf("[wifi] json_create failed (bad JSON or pool too small)\n");
                }
                
                free(mqtt_data_copy);   // Free the allocated memory
            }
        }
        break;
            
        default:
            break;
    }
}

/**
 * @brief Initialize and start MQTT client
 * 
 * This function performs the following operations:
 * 1. Retrieves user ID from NVS storage
 * 2. Dynamically allocates and creates all MQTT topic strings
 * 3. Configures MQTT client with proper LWT topic
 * 4. Initializes and starts the MQTT client
 * 
 * Configures MQTT client with:
 * - HiveMQ public broker
 * - Persistent session
 * - Dynamic Last Will and Testament topic
 * - Keep-alive and timeout settings
 */
esp_err_t mqtt_app_start(void)
{
    // Step 1: Retrieve user ID from NVS storage
    if(user_id == NULL){
        retrieve_user_id();
    }

    assert(user_id);

    // Step 2: Dynamically allocate MQTT topic strings
    // Using snprintf with NULL to calculate required buffer size, then allocate exact memory
    
    // Last Will and Testament topic - published by broker when device goes offline
    if(!lwt_topic){
        size_t lwt_topic_size = snprintf(NULL, 0, "prod/users/%s/devices/lumos/%s/lwt", user_id, device_id);
        lwt_topic = malloc(lwt_topic_size + 1);
        if(!lwt_topic){
            printf("[wifi] Failed to allocate memory for LWT topic\n");
            return ESP_ERR_NO_MEM;
        }
        snprintf(lwt_topic, lwt_topic_size + 1, "prod/users/%s/devices/lumos/%s/lwt", user_id, device_id);
    }

    // Device status topic - for status updates
    if(!status_topic){
        size_t status_topic_size = snprintf(NULL, 0, "prod/users/%s/devices/lumos/%s/status", user_id, device_id);
        status_topic = malloc(status_topic_size + 1);
        if(!status_topic){
            printf("[wifi] Failed to allocate memory for status topic\n");
            return ESP_ERR_NO_MEM;
        }
        snprintf(status_topic, status_topic_size + 1, "prod/users/%s/devices/lumos/%s/status", user_id, device_id);
    }

    // Registration topic - for device management and configuration
    if(!register_topic){
        size_t register_topic_size = snprintf(NULL, 0, "prod/users/%s/devices/lumos/%s/register", user_id, device_id);
        register_topic = malloc(register_topic_size + 1);
        if(!register_topic){
            printf("[wifi] Failed to allocate memory for register topic\n");
            return ESP_ERR_NO_MEM;
        }
        snprintf(register_topic, register_topic_size + 1, "prod/users/%s/devices/lumos/%s/register", user_id, device_id);
    }

    // Command topic - for device commands
    if(!command_topic){
        size_t command_topic_size = snprintf(NULL, 0, "prod/users/%s/devices/lumos/%s/command", user_id, device_id);
        command_topic = malloc(command_topic_size + 1);
        if(!command_topic){
            printf("[wifi] Failed to allocate memory for command topic\n");
            return ESP_ERR_NO_MEM;
        }
        snprintf(command_topic, command_topic_size + 1, "prod/users/%s/devices/lumos/%s/command", user_id, device_id);
    }

    // OTA topic - for OTA updates
    if(!ota_topic){
        size_t ota_topic_size = snprintf(NULL, 0, "prod/users/%s/devices/lumos/%s/ota_update", user_id, device_id);
        ota_topic = malloc(ota_topic_size + 1);
        if(!ota_topic){
            printf("[wifi] Failed to allocate memory for ota topic\n");
            return ESP_ERR_NO_MEM;
        }
        snprintf(ota_topic, ota_topic_size + 1, "prod/users/%s/devices/lumos/%s/ota_update", user_id, device_id);
    }

    // Telemetry topic - for telemetry data
    if(!telemetry_topic){
        size_t telemetry_topic_size = snprintf(NULL, 0, "prod/users/%s/devices/lumos/%s/telemetry", user_id, device_id);
        telemetry_topic = malloc(telemetry_topic_size + 1);
        if(!telemetry_topic){
            printf("[wifi] Failed to allocate memory for telemetry topic\n");
            return ESP_ERR_NO_MEM;
        }
        snprintf(telemetry_topic, telemetry_topic_size + 1, "prod/users/%s/devices/lumos/%s/telemetry", user_id, device_id);
    }

    // Step 3: Configure MQTT client with dynamic LWT topic
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = "mqtt://broker.hivemq.com:1883",
        .session.disable_clean_session = 0,    // Enable persistent session
        .credentials.client_id = device_id,  // Fixed client ID for persistence
        .session.keepalive = 120,      // Standard 120-second keepalive
        .network.timeout_ms = 60000,  // 60 second network timeout
        .session.last_will.topic = lwt_topic,   // Dynamic LWT topic with user_id and device_id
        .session.last_will.msg = "offline",
        .session.last_will.qos = 1,
        .session.last_will.retain = 0,
    };

    // Step 4: Initialize and start MQTT client
    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    ESP_ERROR_CHECK(esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL));
    ESP_ERROR_CHECK(esp_mqtt_client_start(mqtt_client));

    return ESP_OK;
}

esp_err_t mqtt_app_stop(void){
    if(!mqtt_client){
        return ESP_ERR_INVALID_ARG;
    }   
    esp_mqtt_client_stop(mqtt_client);
    esp_mqtt_client_disconnect(mqtt_client);
    esp_mqtt_client_destroy(mqtt_client);
    return ESP_OK;
}

/*==============================================================================
 * WIFI PROVISIONING FUNCTIONS
 *============================================================================*/

/**
 * @brief Custom endpoint handler for receiving user ID during provisioning
 * @param session_id Session ID
 * @param inbuf Input buffer containing user ID
 * @param inlen Input buffer length
 * @param outbuf Output buffer (allocated by this function)
 * @param outlen Output buffer length
 * @param priv_data Private data (unused)
 * @return ESP_OK on success, error code otherwise
 * 
 * This handler receives user ID from the mobile app during provisioning
 * and stores it in NVS for later use in MQTT topics
 */
static esp_err_t user_id_handler(uint32_t session_id, const uint8_t *inbuf, ssize_t inlen,
                            uint8_t **outbuf, ssize_t *outlen, void *priv_data)
{
    if (inbuf == NULL || inlen <= 0) {
        return ESP_ERR_INVALID_ARG;
    }

    // Allocate memory for output buffer
    // Add 1 for null terminator if input isn't null-terminated
    *outbuf = malloc(inlen + 1);
    if (*outbuf == NULL) {
        return ESP_ERR_NO_MEM;
    }

    // Copy received user ID to output buffer
    memcpy(*outbuf, inbuf, inlen);
    (*outbuf)[inlen] = '\0';  // Ensure null termination
    *outlen = inlen + 1;
    
    nvs_open("device_config", NVS_READWRITE, &device_config_h);    // Open the device config namespace from the default NVS partition
    nvs_set_str(device_config_h, "user_id", (char *)*outbuf);       // Set the user ID in the device config namespace
    nvs_commit(device_config_h);   

    return ESP_OK;
}


/*==============================================================================
 * WIFI EVENT HANDLERS
 *============================================================================*/

/**
 * @brief Central event handler for WiFi, IP, and provisioning events
 * @param arg Event arguments (unused)
 * @param event_base Event base (WIFI_EVENT, IP_EVENT, WIFI_PROV_EVENT, etc.)
 * @param event_id Specific event ID
 * @param event_data Event-specific data
 * 
 * Handles all network-related events including:
 * - BLE transport events
 * - WiFi provisioning events
 * - WiFi connection/disconnection events
 * - IP address assignment events
 */
static void event_handler(void* arg, esp_event_base_t event_base,
                          int32_t event_id, void* event_data)
{
    if (event_base == PROTOCOMM_TRANSPORT_BLE_EVENT) {
        switch (event_id) {
            case PROTOCOMM_TRANSPORT_BLE_CONNECTED:
                printf("BLE transport: Device Connected!\n");
                break;
            case PROTOCOMM_TRANSPORT_BLE_DISCONNECTED:
                printf("BLE transport: Device Disconnected!\n");
                break;
            default:
                printf("BLE transport: Event %d received\n", (int)event_id);
                break;
        }
    }

    if (event_base == WIFI_PROV_EVENT) {
        switch (event_id) {
            case WIFI_PROV_START:
                printf("Provisioning started\n");
                break;
            case WIFI_PROV_INIT:
                printf("Provisioning service initialized\n");
                break;
            case WIFI_PROV_DEINIT:
                printf("Provisioning service deinitialized\n");
                break;
            case WIFI_PROV_CRED_RECV: {
                //const wifi_sta_config_t *wifi_sta_cfg = event_data;
                //printf("[wifi] Received Wi-Fi credentials\n\tSSID     : %s\n\tPassword : %s",
                //     wifi_sta_cfg->ssid,
                //     wifi_sta_cfg->password);
                break;
            }
            case WIFI_PROV_CRED_FAIL: {
                //const wifi_prov_sta_fail_reason_t *reason = event_data;
                //printf("[wifi] Provisioning failed!\n\tReason : %s"
                //         "\n\tPlease reset to factory and retry provisioning",
                //         (*reason == WIFI_PROV_STA_AUTH_ERROR) ?
                //         "Wi-Fi station authentication failed" : "Wi-Fi access-point not found");*/

                wifi_prov_mgr_reset_sm_state_for_reprovision();
                break;
            }
                case WIFI_PROV_CRED_SUCCESS:
                xEventGroupSetBits(events_EventGroup, EVENT_NETWORK_PROVISIONED_BIT);
                break;
            case WIFI_PROV_END:
                /* De-initialize manager once provisioning is finished */
                wifi_prov_mgr_deinit();
                break;
            default:
                printf("Provisioning Event: %d\n", (int)event_id);
                break;
        }
    }

    else if(event_base == WIFI_EVENT){
        switch (event_id) {
            case WIFI_EVENT_STA_START:
                esp_wifi_connect();
                break;
            case WIFI_EVENT_STA_CONNECTED:
                printf("Connected to AP\n");
                break;
            case WIFI_EVENT_STA_DISCONNECTED:
                //printf("[wifi] Disconnected. Connecting to the AP again...\n");
                xEventGroupSetBits(events_EventGroup, EVENT_WIFI_DISCONNECTED_BIT);
                break;
        }
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        status = WIFI_SUCCESS;
        xEventGroupSetBits(events_EventGroup, EVENT_WIFI_CONNECTED_BIT);
    }
}

/*==============================================================================
 * WIFI FUNCTIONS
 *============================================================================*/

/**
 * @brief Start WiFi in station mode
 * 
 * Configures and starts WiFi as a station
 * if enabled. This function is called after successful provisioning.
 */


static void wifi_start_sta(void)
{
    /* Start Wi-Fi in station mode */
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    
    ESP_ERROR_CHECK(esp_wifi_start());
}

/**
 * @brief Reconnect to WiFi
 * 
 * Reconnects to WiFi with a backoff strategy
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t wifi_reconnect(void){
    int retry_count = 0;
    esp_wifi_connect();
    while(status == WIFI_FAILURE){
        if(retry_count >= MAX_FAILURES){
            return ESP_FAIL;
        }
        retry_count++;
        printf("[wifi] Reconnecting to WiFi... %d\n", retry_count);
        vTaskDelay(pdMS_TO_TICKS(100 * retry_count));
    }
    return ESP_OK;
}

/**
 * @brief Initialize WiFi provisioning system
 * 
 * Main entry point for network initialization. This function:
 * - Sets up TCP/IP stack and event loops
 * - Initializes WiFi subsystem
 * - Configures BLE-based provisioning manager
 * - Checks if device is already provisioned
 * - Either starts normal WiFi operation or provisioning service
 * - Registers custom endpoint for user ID exchange
 */
void init_wifi_provisioning(void){
    // Initialize TCP/IP 
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());   // To handle WiFi events

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // Initialize Wi-Fi including netif with default config 
    esp_netif_create_default_wifi_sta();


    // Register our event handler for Wi-Fi, IP and Provisioning related events 
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_PROV_EVENT, ESP_EVENT_ANY_ID, event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, event_handler, NULL));
    

    // Initialize provisioning manager 
    wifi_prov_mgr_config_t config = {
        .scheme = wifi_prov_scheme_ble,
        .scheme_event_handler = WIFI_PROV_SCHEME_BLE_EVENT_HANDLER_FREE_BTDM
    };
    ESP_ERROR_CHECK(wifi_prov_mgr_init(config));

    // Check if device is provisioned
    bool provisioned = false;
    ESP_ERROR_CHECK(wifi_prov_mgr_is_provisioned(&provisioned));

    // If device is not yet provisioned start provisioning service 
    if (provisioned) {
        xEventGroupSetBits(events_EventGroup, EVENT_NETWORK_PROVISIONED_BIT);
        wifi_prov_mgr_deinit();
        
        wifi_start_sta();
    }
    else {
        printf("[wifi] Device is not provisioned, starting provisioning service\n");
    
        ESP_ERROR_CHECK(wifi_prov_mgr_endpoint_create("custom-data"));

        // Register app information for proto-ver endpoint
        const char *capabilities[] = {"custom-data" };  // List your custom endpoints
        esp_err_t ret = wifi_prov_mgr_set_app_info("myapp", "v1.0", capabilities, 1);
        if (ret != ESP_OK) {
            printf("[wifi] Failed to set app info, error: %d\n", ret);
        }

        // Small delay to ensure registration is complete
        vTaskDelay(pdMS_TO_TICKS(100));

        ESP_ERROR_CHECK(wifi_prov_mgr_start_provisioning(security, NULL, device_type, NULL));
        printf("[wifi] Starting provisioning service\n");
        ESP_ERROR_CHECK(wifi_prov_mgr_endpoint_register("custom-data", user_id_handler, NULL));
    }
}
