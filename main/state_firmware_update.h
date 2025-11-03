#ifndef STATE_FIRMWARE_UPDATE_H
#define STATE_FIRMWARE_UPDATE_H

#include "esp_err.h"
#include "state_common.h"
#include "stdint.h"

/**
 * @brief Entry point for the firmware update state
 * 
 * Initializes the firmware update process including:
 * - Setting up HTTPS OTA configuration
 * - Starting the OTA download process
 * - Handling OTA progress and completion
 * 
 * @param data Pointer to common state data structure
 * @return esp_err_t ESP_OK on success, error code on failure
 */
esp_err_t firmware_update_entry(struct state_common_data *data);

/**
 * @brief Exit point for the firmware update state
 * 
 * Cleans up firmware update resources and handles post-OTA operations
 * 
 * @param data Pointer to common state data structure
 * @return esp_err_t ESP_OK on success, error code on failure
 */
esp_err_t firmware_update_exit(struct state_common_data *data);

/**
 * @brief Event handler for the firmware update state
 * 
 * Processes OTA-specific events:
 * - OTA progress updates
 * - OTA completion
 * - OTA errors and recovery
 * 
 * @param event Event type identifier
 * @param data Pointer to common state data structure
 * @return esp_err_t ESP_OK on success, error code on failure
 */
esp_err_t firmware_update_event(uint8_t event, struct state_common_data *data);

#endif  // STATE_FIRMWARE_UPDATE_H
