/**
 * @file main.c
 * @brief Main application entry point for ESP32 2-Gang Smart Switch
 * 
 * This module serves as a Gateway to enable smart functionality for the Renesas switch panel.
 * Key features include:
 * - Heartbeat detection with Renesas board
 * - Product information and status querying
 * - Network connection management (WiFi + MQTT)
 * - State machine coordination
 * - MCU communication via UART
 * 
 * @author Smart Switch Team
 * @date 2024
 */

/*==============================================================================
 * INCLUDES
 *============================================================================*/

// Custom includes
#include "config.h"
#include "state_machine.h"
#include "uart_comm.h"

// ESP-IDF includes
#include "esp_err.h"
#include "nvs_flash.h"
#include "esp_mac.h"

/*==============================================================================
 * DEFINES AND CONSTANTS
 *============================================================================*/

/*==============================================================================
 * GLOBAL VARIABLES
 *============================================================================*/

// Device configuration constants
const char* device_type = "lumos_3g1f";                    // Device type identifier
char* device_id = NULL;                            // Device ID
static uint8_t default_device_id[6];               // Base MAC address from EFUSE

/*==============================================================================
 * PRIVATE FUNCTION DECLARATIONS
 *============================================================================*/

static esp_err_t init_nvs_flash(void);
static esp_err_t init_uart_communication(void);
static void retrieve_deviceID(void);

/*==============================================================================
 * PRIVATE FUNCTION IMPLEMENTATIONS
 *============================================================================*/

/**
 * @brief Initialize NVS flash storage
 * @return ESP_OK on success, error code otherwise
 * 
 * Handles NVS initialization with automatic erase if needed
 */
static esp_err_t init_nvs_flash(void)
{
    esp_err_t ret = nvs_flash_init();
    
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        printf("[main] NVS partition needs to be erased\n");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    
    return ret;
}

/**
 * @brief Initialize UART communication with MCU
 * @return ESP_OK on success, error code otherwise
 */
static esp_err_t init_uart_communication(void)
{
    esp_err_t ret = uart_init();
    if (ret != ESP_OK) {
        printf("[main] UART communication initialization failed: %s\n", esp_err_to_name(ret));
    }
    
    return ret;
}

/**
 * @brief Retrieve the device ID from NVS
 * @return ESP_OK on success, error code otherwise
 */
static void retrieve_deviceID(void)
{
    esp_efuse_mac_get_default(default_device_id);
    device_id = (char *)malloc(sizeof(default_device_id) * 2 + 1);
    if(!device_id){
        printf("[main] Failed to allocate memory for device ID\n");
        return;
    }
    snprintf(device_id, (sizeof(default_device_id) * 2 + 1), "%02x%02x%02x%02x%02x%02x", default_device_id[0], default_device_id[1], default_device_id[2], default_device_id[3], default_device_id[4], default_device_id[5]);
}

/*==============================================================================
 * PUBLIC FUNCTION IMPLEMENTATIONS
 *============================================================================*/

/**
 * @brief Main application entry point
 * 
 * Initializes all subsystems and starts the main state machine:
 * 1. NVS flash storage initialization
 * 2. UART communication setup
 * 3. State machine execution
 */
void app_main(void)
{
    // Retrieve the default MAC address from EFUSE
    retrieve_deviceID();
    printf("[main] Device ID: %s\n", device_id);

    // Initialize NVS flash storage
    if (init_nvs_flash() != ESP_OK) {
        printf("[main] Failed to initialize NVS flash\n");
        return;
    }
    
    // Initialize UART communication
    if (init_uart_communication() != ESP_OK) {
        printf("[main] Failed to initialize UART communication\n");
        return;
    }
    state_machine_run();
}
