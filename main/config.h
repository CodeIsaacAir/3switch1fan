#ifndef CONFIG_H
#define CONFIG_H

#include "stdint.h"


// Device ID
extern char* device_id;     // Base MAC Address of the device retrieved from EFUSE

// Device Type
extern const char *device_type;

// MQTT topics 
extern char* lwt_topic;
extern char* status_topic;
extern char* register_topic;
extern char* command_topic;
extern char* ota_topic;
extern char* telemetry_topic;

// OTA firmware URL
extern char firmware_url[512];

// Maximum buffer size for various buffers
#define UART_BUFFER_SIZE 80
#define MQTT_STATUS_BUFFER_SIZE 32
#define MQTT_COMMAND_BUFFER_SIZE 64

#endif