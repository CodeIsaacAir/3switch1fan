#ifndef STATE_OPERATIONAL_H
#define STATE_OPERATIONAL_H

#include "esp_err.h"
#include "state_common.h"
#include "stdint.h"

esp_err_t operational_entry(struct state_common_data *data);
esp_err_t operational_exit(struct state_common_data *data);
esp_err_t operational_event(uint8_t event, struct state_common_data *data);
esp_err_t update_datapoint_status(uint8_t gang_datapointID, uint8_t gang_action);

#endif  //STATE_OPERATIONAL_H