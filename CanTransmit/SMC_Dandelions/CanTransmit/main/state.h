#ifndef STATE_H
#define STATE_H



typedef enum {
    BOOT,
    IDLE,
    EXPERIMENT,
    NEUTRAL,
    LOG,
    COMMS,
    SHUTDOWN
} state_t;

// DANDELIONS FUNCTION PROTOYPES- 
void state_machine(void);

#endif
