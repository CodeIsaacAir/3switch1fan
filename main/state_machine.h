#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include "state_common.h"
#include "time.h"

/* Custom Data Structures definitions*/
struct state_machine_data{
    main_state_t main_state;
    struct state_common_data common;
};

/* Function Prototypes */
// Implements an overarching state machine for the application
void state_machine_run(void);

extern esp_reset_reason_t rstReason;
extern time_t rstTimestamp;

#endif