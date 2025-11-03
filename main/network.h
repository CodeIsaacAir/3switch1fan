#ifndef NETWORK_H
#define NETWORK_H

#include "mqtt_client.h"
#include "state_common.h"


// WIFI EVENT BIT MASKS
#define WIFI_SUCCESS (1 << 0) // 0000 0001 = 1 (00000001)
#define WIFI_FAILURE (1 << 1) // 0000 0010 = 2 (00000010)
#define MAX_FAILURES 10     // Maximum number of retries to connect to WiFi

// MQTT Client Handle
extern esp_mqtt_client_handle_t mqtt_client;

void init_wifi_provisioning(void);
void retrieve_user_id(void);
esp_err_t mqtt_app_start(void);
esp_err_t mqtt_app_stop(void);
esp_err_t register_device(dpConfig_t dpConfig);
esp_err_t wifi_reconnect(void); // with backoff



#endif  //NETWORK_H