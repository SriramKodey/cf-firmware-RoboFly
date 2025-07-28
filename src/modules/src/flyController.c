#include "config.h"
#include "debug.h"
#include "FreeRTOS.h"
#include "system.h"
#include "static_mem.h"
#include "task.h"
#include "log.h"

#include "flyController.h"
#include "disc_spi.h"

#define ID_FOR_FSM_CONTROL ((uint32_t)912345678)
#define MAX_TIME_IN_LAND 500  // in ms

/* File Static Varirables */
static xQueueHandle inputQueue;
STATIC_MEM_QUEUE_ALLOC(inputQueue, 5, sizeof(flyState_t));

/* flyController struct - needs to be static!! */
static const float liftoffAmp = 140.0;
static const float offsetV = 130;

static flyState_t state;
static flyController_PID_t flyController;
static desriedPosition_t setPoint;
static desriedPosition_t initPoint;
static uint32_t counter = 1;
static uint32_t notification = 0;
static QueueHandle_t spiTaskQueueHandle;
static TaskHandle_t flyControllerTaskHandle;
static ControllerMode_t flyControllerMode;
static TickType_t xTaskWaitTime = portMAX_DELAY;
static TickType_t xLastWakeTime;
static TickType_t xTimeLandModeStart;

static float delta_amp = 0.0;
static float delta_offset = 0.0;

/* Task prototype and Stack allocation */
static void flyControllerTask(void *);
STATIC_MEM_TASK_ALLOC(flyControllerTask, CONTROLLER_TASK_STACKSIZE);

static bool isInit = false;

flyControllerHandles_t flyControllerTaskInit(QueueHandle_t sendQueue) {
    inputQueue = STATIC_MEM_QUEUE_CREATE(inputQueue);

    spiTaskQueueHandle = sendQueue;
    /* Initialise flyController */
    flyController_PID_Init(&flyController);
    // Controller settings
    // flyController.p1.attitude_ON = false;
    flyController.p1.lateral_ON = false;

    flyController.p1.liftoff_V = liftoffAmp;
    flyController.p1.offset_V = offsetV;
    // Controller settings

    flyControllerMode = MODE_IDLE;

    initPoint.X = 0;
    initPoint.Y = 0;
    initPoint.Z = 0;

    flyControllerTaskHandle = STATIC_MEM_TASK_CREATE(flyControllerTask, flyControllerTask, CONTROLLER_TASK_NAME, NULL, CONTROLLER_TASK_PRI);
    isInit = true;

    flyControllerHandles_t flyControllerHandles;
    // Set the handles
    flyControllerHandles.flyControllerTaskHandle = flyControllerTaskHandle;
    flyControllerHandles.flyControllerQueueHandle = inputQueue;

    return flyControllerHandles;
}

bool flyControllerTaskTest() {
    return isInit;
}

static void flyControllerTask(void* parameters) {
    systemWaitStart();
    while(true) {

        // Receive the mode from the State Machine
        if (pdTRUE == xTaskNotifyWait(0x00, 0xFFFFFFFF, &notification, 0)) {
            flyControllerMode = (ControllerMode_t) notification;

            if (flyControllerMode == MODE_LIFTOFF) {
                setPoint.X = initPoint.X;
                setPoint.Y = initPoint.Y;
                setPoint.Z = initPoint.Z;
            }

            if (flyControllerMode == MODE_CONTROL_ON) {
                flyController.p1.lateral_ON = true; // Switch on lateral controller
                setPoint.Z = setPoint.Z + 0.1f; // Increment Z by 0.1 for control on
            }

            if (flyControllerMode == MODE_LAND) {
                xTaskWaitTime = pdMS_TO_TICKS(0);
                xLastWakeTime = xTaskGetTickCount();
                xTimeLandModeStart = xTaskGetTickCount();
                delta_amp = flyController.output.delta_amplitude;
                delta_offset = flyController.output.delta_offset;
            }

            if (flyControllerMode == MODE_RESET) {
                xTaskWaitTime = portMAX_DELAY;
            }
        }

        if (pdTRUE == xQueueReceive(inputQueue, &state, xTaskWaitTime) || (flyControllerMode == MODE_LAND)) {
            // Set StateID for copying
            // Call the Filter
            flyController.output.ID = state.ID;
            state = filter_state(&flyController, state);

            switch (flyControllerMode) {
                case MODE_IDLE:
                    initPoint.X = (initPoint.X*counter + state.positionX) / (counter + 1);
                    initPoint.Y = (initPoint.Y*counter + state.positionY) / (counter + 1);
                    initPoint.Z = (initPoint.Z*counter + state.altitudeZ) / (counter + 1);
                    ++counter;
                    break;

                case MODE_LIFTOFF:
                    control(&flyController, state, setPoint);
                    /* Enqueue control */
                    xQueueSend(spiTaskQueueHandle, &(flyController.output), 0);
                    break;


                case MODE_CONTROL_ON:
                    control(&flyController, state, setPoint);
                    /* Enqueue control */
                    xQueueSend(spiTaskQueueHandle, &(flyController.output), 0);
                    break;

                case MODE_LAND:
                    // ramp down
                    flyController.output.ID = ID_FOR_FSM_CONTROL;
                    float scale = 1.0f - ((float)(xTaskGetTickCount() - xTimeLandModeStart) / (float)MAX_TIME_IN_LAND);
                    scale = fmaxf(0.0f, fminf(1.0f, scale)); // clamp between 0 and 1

                    flyController.output.delta_amplitude = delta_amp * scale;
                    flyController.output.delta_offset = delta_offset * scale;
                    
                    xQueueSend(spiTaskQueueHandle, &(flyController.output), 0);
                    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(4)); 
                    break;

                case MODE_RESET:
                    // Reset for restart
                    // Build Later
                    break;

                case MODE_NONE:
                    break;
            }
        }
    }
}

void flyControllerTaskEnqueueInput(flyState_t state) {
    xQueueOverwrite(inputQueue, &state);
}

