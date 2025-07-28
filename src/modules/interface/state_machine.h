#ifndef __STATE_MACHINE_H__
#define __STATE_MACHINE_H__

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

typedef enum {
    STATE_IDLE,
    STATE_OFFSET_RAMP,
    STATE_AMP_RAMP,
    STATE_LIFTOFF,
    STATE_CONTROL_ON,
    STATE_LAND,
    STATE_AMP_RAMP_DOWN,
    STATE_OFFSET_RAMP_DOWN,
    STATE_RESET,
    STATE_ABORT
}FSM_State_t;

typedef struct {    
    TaskHandle_t flyControllerTaskHandle;
    QueueHandle_t spiTaskQueueHandle;
} stateMachineHandleInput_t;

typedef struct
{
    /* data */
    FSM_State_t currentState;
    FSM_State_t previousState;
    FSM_State_t lastState;

    TickType_t timeCurrentStateStart;
    TickType_t timeInCurrentState;
} stateMachine_t;

QueueHandle_t state_machineTaskInit (stateMachineHandleInput_t stateMachineHandleInput);

void ChangeState(FSM_State_t newState, ControllerMode_t modeToNotify);

bool state_machineTaskTest();

#endif