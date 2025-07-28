#include "config.h"
#include "debug.h"
#include "FreeRTOS.h"
#include "static_mem.h"
#include "task.h"
#include "deck.h"
#include "deck_constants.h"

#include "flyController.h"
#include "flyController_PID.h"
#include "state_machine.h"
#include "system.h"

#define ID_FOR_FSM_CONTROL ((uint32_t)912345678)
#define MAX_TIME_IN_OFFSET_RAMP       pdMS_TO_TICKS(2000) // in ms
#define MAX_TIME_IN_AMP_RAMP          pdMS_TO_TICKS(2000) // in ms
#define MAX_TIME_IN_LIFTOFF           pdMS_TO_TICKS(100)  // in ms
#define MAX_TIME_IN_LAND              pdMS_TO_TICKS(500)  // in ms
#define MAX_TIME_IN_AMP_RAMP_DOWN     pdMS_TO_TICKS(2000) // in ms
#define MAX_TIME_IN_OFFSET_RAMP_DOWN  pdMS_TO_TICKS(2000) // in ms
#define MAX_TIME_IN_RESET             pdMS_TO_TICKS(1000) // in ms

#define BIAS_VOLTAGE 260
#define OFFSET_BASELINE BIAS_VOLTAGE/2
#define AMP_LIFTOFF 140
#define AMP_LANDING AMP_LIFTOFF - 20

static xQueueHandle inputQueue;
STATIC_MEM_QUEUE_ALLOC(inputQueue, 1, sizeof(char));

static TaskHandle_t flyControllerTaskHandle;
static QueueHandle_t spiTaskQueueHandle;
static deckPin_t bias_Pin;

static void state_machineTask(void*);
STATIC_MEM_TASK_ALLOC(state_machineTask, STATE_MACHINE_TASK_STACKSIZE);

static char code; // static or volatile
static stateMachine_t stateMachine;
static flyControl_t flyControlOutput;
static bool isInit = false;

void ChangeState(FSM_State_t newState, ControllerMode_t modeToNotify)
{
    stateMachine.previousState = stateMachine.currentState;
    stateMachine.currentState = newState;
    stateMachine.timeCurrentStateStart = xTaskGetTickCount();

    if (modeToNotify != MODE_NONE)
    {
        xTaskNotify(flyControllerTaskHandle, (uint32_t) modeToNotify, eSetValueWithOverwrite);
    }
}

QueueHandle_t state_machineTaskInit(stateMachineHandleInput_t stateMachineHandleInput) {
    inputQueue = STATIC_MEM_QUEUE_CREATE(inputQueue);

    flyControllerTaskHandle = stateMachineHandleInput.flyControllerTaskHandle;
    spiTaskQueueHandle = stateMachineHandleInput.spiTaskQueueHandle;

    stateMachine.currentState = STATE_IDLE;
    stateMachine.previousState = STATE_IDLE;

    bias_Pin = DECK_GPIO_IO1;
    pinMode(bias_Pin, OUTPUT);

    flyControlOutput.offset = 0.0;
    flyControlOutput.amplitude = 0.0;
    flyControlOutput.delta_amplitude = 0.0;
    flyControlOutput.offset = 0.0;
    flyControlOutput.ID = ID_FOR_FSM_CONTROL;

    STATIC_MEM_TASK_CREATE(state_machineTask, state_machineTask, STATE_MACHINE_TASK_NAME, NULL, STATE_MACHINE_TASK_PRI);
    isInit = true;

    return inputQueue;
}

bool state_machineTaskTest() {
    return isInit;
}

static void state_machineTask(void* parameters) {
    systemWaitStart();
    stateMachine.timeCurrentStateStart = xTaskGetTickCount();
    //xLastWakeTime = xTaskGetTickCount();

    while (true) {
        // Wait for input from flyControllerTask
        if (pdTRUE == xQueueReceive(inputQueue, &code, 0)) {
            if (code == 's') {
                if (stateMachine.previousState == STATE_IDLE) {
                    ChangeState(STATE_OFFSET_RAMP, MODE_NONE);
                    // turn on GPIO to Trigger Bias//
                    digitalWrite(bias_Pin, HIGH);
                }
            }

            if (code == 'e') {
                if (stateMachine.currentState == STATE_CONTROL_ON) {
                    ChangeState(STATE_LAND, MODE_LAND);
                }
            }
        }

        stateMachine.timeInCurrentState = xTaskGetTickCount() - stateMachine.timeCurrentStateStart;

        switch (stateMachine.currentState) {
            case STATE_IDLE:
                break;

            case STATE_OFFSET_RAMP:
                if (stateMachine.timeInCurrentState >= MAX_TIME_IN_OFFSET_RAMP) {
                    ChangeState(STATE_AMP_RAMP, MODE_IDLE);
                    break;
                }
                // Should send spi commands
                flyControlOutput.offset = (float) OFFSET_BASELINE * ((float) stateMachine.timeInCurrentState / (float) MAX_TIME_IN_OFFSET_RAMP);
                xQueueSend(spiTaskQueueHandle, &flyControlOutput, 0);
                break;

            case STATE_AMP_RAMP:
                if (stateMachine.timeInCurrentState >= MAX_TIME_IN_AMP_RAMP) {
                    ChangeState(STATE_LIFTOFF, MODE_LIFTOFF);
                    break;
                }
                // Should send spi commands
                flyControlOutput.amplitude = (float) AMP_LIFTOFF * ((float) stateMachine.timeInCurrentState / (float) MAX_TIME_IN_AMP_RAMP);
                xQueueSend(spiTaskQueueHandle, &flyControlOutput, 0);
                break;

            case STATE_LIFTOFF:
                if (stateMachine.timeInCurrentState >= MAX_TIME_IN_LIFTOFF) {
                    ChangeState(STATE_CONTROL_ON, MODE_CONTROL_ON);
                    break;
                }
                break;

            case STATE_CONTROL_ON:
                break;

            case STATE_LAND:
                if (stateMachine.timeInCurrentState >= MAX_TIME_IN_LAND) {
                    ChangeState(STATE_AMP_RAMP_DOWN, MODE_RESET);
                    break;
                }
                break;

            case STATE_AMP_RAMP_DOWN:
                if (stateMachine.timeInCurrentState >= MAX_TIME_IN_AMP_RAMP_DOWN) {
                    ChangeState(STATE_OFFSET_RAMP_DOWN, MODE_NONE);
                    digitalWrite(bias_Pin, LOW);
                    break;
                }
                flyControlOutput.amplitude = (float) AMP_LIFTOFF * ((float) 1 - ((float) stateMachine.timeInCurrentState / (float) MAX_TIME_IN_AMP_RAMP));
                xQueueSend(spiTaskQueueHandle, &flyControlOutput, 0);
                break;

            case STATE_OFFSET_RAMP_DOWN:
                if (stateMachine.timeInCurrentState >= MAX_TIME_IN_OFFSET_RAMP_DOWN) {
                    ChangeState(STATE_RESET, MODE_NONE);
                    break;
                }
                flyControlOutput.offset = (float) OFFSET_BASELINE * ((float) 1 - ((float) stateMachine.timeInCurrentState / (float) MAX_TIME_IN_OFFSET_RAMP));
                xQueueSend(spiTaskQueueHandle, &flyControlOutput, 0);
                break;

            case STATE_RESET:
                if (stateMachine.timeInCurrentState >= MAX_TIME_IN_RESET) {
                    ChangeState(STATE_IDLE, MODE_NONE);
                    break;
                }
                break;

            case STATE_ABORT:
                break; // deal later
        }

        vTaskDelay(pdMS_TO_TICKS(4)); // Think about offseting a bit
    }
}

void stateMachineTaskEnqueueInput(char code) {
    xQueueOverwrite(inputQueue, &code);
}