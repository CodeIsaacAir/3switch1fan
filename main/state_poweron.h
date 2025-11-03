#ifndef STATE_POWERON
#define STATE_POWERON

#include "state_common.h"


esp_err_t powerOn_entry(struct state_common_data *data);
esp_err_t powerOn_exit(struct state_common_data *data);

#endif 