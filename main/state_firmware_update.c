/**
 * @file state_firmware_update.c
 * @brief Firmware update state implementation for the 2-gang switch
 * 
 * This file handles the firmware update state of the device, including:
 * - HTTPS OTA initialization and configuration
 * - Firmware download and verification
 * - OTA progress monitoring and error handling
 * - Post-OTA restart and recovery
 */

//==============================================================================
// INCLUDES
//==============================================================================

// Custom includes
#include "state_firmware_update.h"
#include "state_common.h"
#include "config.h"

// ESP-IDF includes
#include "esp_err.h"
#include "esp_https_ota.h"
#include "esp_http_client.h"
#include "esp_app_desc.h"
#include "esp_system.h"
#include "esp_ota_ops.h"
#include "freertos/task.h"

//==============================================================================
// CONSTANTS AND STATIC VARIABLES
//==============================================================================

static struct state_common_data* data_lifetime;              // Pointer to persistent state data
static TaskHandle_t ota_task_handle = NULL;                  // Handle for OTA task

// Server certificate for HTTPS verification (placeholder - should be updated with actual cert)
//extern const uint8_t server_cert_pem_start[] asm("_binary_server_cert_pem_start");
//extern const uint8_t server_cert_pem_end[] asm("_binary_server_cert_pem_end");

//==============================================================================
// STATIC FUNCTION DECLARATIONS
//==============================================================================

static void ota_task(void *pvParameter);
static esp_err_t validate_image_header(esp_https_ota_handle_t https_ota_handle);
static void ota_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);

//==============================================================================
// STATIC FUNCTION IMPLEMENTATIONS
//==============================================================================

/**
 * @brief OTA event handler for system events
 * 
 * Handles various OTA system events and updates the state machine accordingly
 * 
 * @param arg User argument (unused)
 * @param event_base Event base
 * @param event_id Event ID
 * @param event_data Event data
 */
static void ota_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_base == ESP_HTTPS_OTA_EVENT) {
        switch (event_id) {
            case ESP_HTTPS_OTA_START:
                printf("[firmware_update] OTA started\n");
                break;
            case ESP_HTTPS_OTA_CONNECTED:
                printf("[firmware_update] Connected to server\n");
                break;
            case ESP_HTTPS_OTA_GET_IMG_DESC:
                printf("[firmware_update] Reading Image Description\n");
                break;
            case ESP_HTTPS_OTA_VERIFY_CHIP_ID:
                printf("[firmware_update] Verifying chip id of new image\n");
                break;
            case ESP_HTTPS_OTA_DECRYPT_CB:
                printf("[firmware_update] Callback to decrypt function\n");
                break;
            case ESP_HTTPS_OTA_WRITE_FLASH:
                printf("[firmware_update] Writing to flash\n");
                break;
            case ESP_HTTPS_OTA_UPDATE_BOOT_PARTITION:
                printf("[firmware_update] Boot partition updated\n");
                break;
            case ESP_HTTPS_OTA_FINISH:
                printf("[firmware_update] OTA finished\n");
                xEventGroupSetBits(events_EventGroup, EVENT_OTA_COMPLETE_BIT);
                break;
            case ESP_HTTPS_OTA_ABORT:
                printf("[firmware_update] OTA aborted\n");
                xEventGroupSetBits(events_EventGroup, EVENT_OTA_ERROR_BIT);
                break;
        }
    }
}

/**
 * @brief Validate the OTA image header
 * 
 * Checks if the new firmware image is valid and compatible with the current hardware
 * 
 * @param https_ota_handle Handle to the OTA process
 * @return esp_err_t ESP_OK if valid, error code otherwise
 */
static esp_err_t validate_image_header(esp_https_ota_handle_t https_ota_handle) {
    esp_app_desc_t new_app_info;
    esp_err_t err = esp_https_ota_get_img_desc(https_ota_handle, &new_app_info);
    if (err != ESP_OK) {
        printf("[firmware_update] esp_https_ota_read_img_desc failed\n");
        return err;
    }
    
    printf("[firmware_update] New firmware version: %s\n", new_app_info.version);
    
    const esp_app_desc_t *running_app_info = esp_app_get_description();
    printf("[firmware_update] Running firmware version: %s\n", running_app_info->version);
    
    // Check if the new version is different from the current one
    if (memcmp(new_app_info.version, running_app_info->version, sizeof(new_app_info.version)) == 0) {
        printf("[firmware_update] Current running version is the same as a new. We will not continue the update.\n");
        return ESP_FAIL;
    }
    
    return ESP_OK;
}

/**
 * @brief Main OTA task implementation
 * 
 * Performs the actual firmware download and update process
 * 
 * @param pvParameter Task parameter (firmware URL as string)
 */
static void ota_task(void *pvParameter) {
    printf("OTA URL: %s\n", firmware_url);
    
    esp_http_client_config_t config = {
        .url = firmware_url,
        .timeout_ms = 30000,
        .keep_alive_enable = true,
    };
    
    // Add certificate verification if available
    #ifdef CONFIG_OTA_ALLOW_HTTP
    // For testing purposes, allow HTTP (insecure)
    config.skip_cert_common_name_check = true;
    #else
    // For production, use HTTPS with certificate verification
    //config.cert_pem = (char *)server_cert_pem_start;
    #endif
    
    esp_https_ota_config_t ota_config = {
        .http_config = &config,
        .bulk_flash_erase = true
    };
    
    esp_https_ota_handle_t https_ota_handle = NULL;
    esp_err_t err = esp_https_ota_begin(&ota_config, &https_ota_handle);
    if (err != ESP_OK) {
        printf("[firmware_update] ESP HTTPS OTA Begin failed: %s\n", esp_err_to_name(err));
        xEventGroupSetBits(events_EventGroup, EVENT_OTA_ERROR_BIT);
        goto ota_end;
    }
    
    // Validate the image header
    err = validate_image_header(https_ota_handle);
    if (err != ESP_OK) {
        printf("[firmware_update] Image header validation failed\n");
        xEventGroupSetBits(events_EventGroup, EVENT_OTA_ERROR_BIT);
        goto ota_end;
    }
    
    // Perform the OTA update
    int total_len = esp_https_ota_get_image_size(https_ota_handle);
    printf("Image size: %d bytes\n", total_len);
    
    while (1) {
        err = esp_https_ota_perform(https_ota_handle);
        if (err != ESP_ERR_HTTPS_OTA_IN_PROGRESS) {
            break;
        }
        
        // Log progress
        int downloaded = esp_https_ota_get_image_len_read(https_ota_handle);
        if (total_len > 0) {
            int progress = (downloaded * 100) / total_len;
            printf("[firmware_update] OTA Progress: %d%% (%d/%d bytes)\n", progress, downloaded, total_len);
            

        } else {
            printf("[firmware_update] OTA Progress: %d bytes downloaded\n", downloaded);
        }
        
        // Small delay to prevent watchdog timeout
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    if (esp_https_ota_is_complete_data_received(https_ota_handle) != true) {
        // the OTA image was not completely received and user can customise the response to this situation.
        printf("[firmware_update] Complete data was not received.\n");
        xEventGroupSetBits(events_EventGroup, EVENT_OTA_ERROR_BIT);
    } else {
        err = esp_https_ota_finish(https_ota_handle);
        if (err == ESP_OK) {
            printf("OTA Succeed, Rebooting...\n");
            xEventGroupSetBits(events_EventGroup, EVENT_OTA_COMPLETE_BIT);
            vTaskDelay(pdMS_TO_TICKS(1000));
            esp_restart();
        } else if (err == ESP_ERR_OTA_VALIDATE_FAILED) {
            printf("[firmware_update] Image validation failed, image is corrupted\n");
            xEventGroupSetBits(events_EventGroup, EVENT_OTA_ERROR_BIT);
        } else {
            printf("[firmware_update] ESP HTTPS OTA Finish failed: %s\n", esp_err_to_name(err));
            xEventGroupSetBits(events_EventGroup, EVENT_OTA_ERROR_BIT);
        }
    }

ota_end:
    if (https_ota_handle) {
        esp_https_ota_abort(https_ota_handle);
    }
    printf("[firmware_update] OTA task completed with error\n");
    vTaskDelete(NULL);
}

//==============================================================================
// PUBLIC FUNCTION IMPLEMENTATIONS
//==============================================================================

/**
 * @brief Entry point for the firmware update state
 * 
 * Initializes the firmware update process by:
 * 1. Setting up persistent data pointer
 * 2. Registering OTA event handlers
 * 3. Starting the OTA task
 * 
 * @param data Pointer to common state data structure
 * @return esp_err_t ESP_OK on success, error code on failure
 */
esp_err_t firmware_update_entry(struct state_common_data* data) {
    
    data_lifetime = data;  // Store pointer for task access
    
    // Register OTA event handler
    esp_err_t err = esp_event_handler_register(ESP_HTTPS_OTA_EVENT, ESP_EVENT_ANY_ID, &ota_event_handler, NULL);
    if (err != ESP_OK) {
        printf("[firmware_update] Failed to register OTA event handler: %s\n", esp_err_to_name(err));
        return err;
    }
    
    // Create the OTA task
    BaseType_t task_created = xTaskCreatePinnedToCore(
        ota_task,
        "ota_task",
        8192,  // Stack size - OTA requires more stack
        NULL,  // Task parameter (firmware URL can be passed here)
        5,     // Priority
        &ota_task_handle,
        0      // Core 0
    );
    
    if (task_created != pdTRUE) {
        printf("[firmware_update] Failed to create OTA task\n");
        esp_event_handler_unregister(ESP_HTTPS_OTA_EVENT, ESP_EVENT_ANY_ID, &ota_event_handler);
        return ESP_FAIL;
    }
    return ESP_OK;
}

/**
 * @brief Exit point for the firmware update state
 * 
 * Cleans up firmware update resources
 * 
 * @param data Pointer to common state data structure
 * @return esp_err_t ESP_OK on success, error code on failure
 */
esp_err_t firmware_update_exit(struct state_common_data* data) {
    
    // Unregister OTA event handler
    esp_event_handler_unregister(ESP_HTTPS_OTA_EVENT, ESP_EVENT_ANY_ID, &ota_event_handler);
    
    // Delete OTA task if it's still running
    if (ota_task_handle != NULL) {
        vTaskDelete(ota_task_handle);
        ota_task_handle = NULL;
    }
    return ESP_OK;
}

/**
 * @brief Event handler for the firmware update state
 * 
 * Processes OTA-specific events:
 * - OTA completion: Restart the system
 * - OTA errors: Return to operational state
 * 
 * @param event Event type identifier
 * @param data Pointer to common state data structure
 * @return esp_err_t ESP_OK on success, error code on failure
 */
esp_err_t firmware_update_event(uint8_t event, struct state_common_data* data) {
    switch(event) {
        case EVENT_OTA_COMPLETE:
            esp_restart();
            break;
            
        default:
            printf("Unhandled event in firmware update state: %d\n", event);
            break;
    }
    
    return ESP_OK;
}
