#ifndef __FSM_H__
#define __FSM_H__

typedef enum
{
    STATE_INIT,
    STATE_UR5_HOME,
    STATE_UR5_LOAD,
    STATE_UR5_UNLOAD,
    STATE_END
} State_t;

typedef struct
{
    State_t state;
    void (*state_function)(void);
} StateMachine_t;

#endif