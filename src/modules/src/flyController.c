#include "config.h"
#include "debug.h"
#include "FreeRTOS.h"
#include "static_mem.h"
#include "task.h"
#include "log.h"

#include "flyController.h"
#include "disc_spi.h"

/* File Static Varirables */
static xQueueHandle inputQueue;
STATIC_MEM_QUEUE_ALLOC(inputQueue, 5, sizeof(flyState_t));

/* flyController struct - needs to be static!! */
static flyController_PID_t flyController;
static desriedPosition_t setPoint;
static desriedPosition_t initPoint;
static uint32_t counter = 1;
static QueueHandle_t spiTaskQueueHandle;

/* Task prototype and Stack allocation */
static void flyControllerTask(void *);
STATIC_MEM_TASK_ALLOC(flyControllerTask, CONTROLLER_TASK_STACKSIZE);

static bool isInit = false;

xQueueHandle flyControllerTaskInit(QueueHandle_t sendQueue) {
    inputQueue = STATIC_MEM_QUEUE_CREATE(inputQueue);

    spiTaskQueueHandle = sendQueue;
    /* Initialise flyController */
    flyController_PID_Init(&flyController);
    // Controller Debugging
    // flyController.p1.attitude_ON = false;
    // flyController.p1.lateral_ON = false;
    // Controller Debugging

    initPoint.X = 0;
    initPoint.Y = 0;
    initPoint.Z = 0;

    STATIC_MEM_TASK_CREATE(flyControllerTask, flyControllerTask, CONTROLLER_TASK_NAME, NULL, CONTROLLER_TASK_PRI);
    isInit = true;

    return inputQueue;
}

bool flyControllerTaskTest() {
    return isInit;
}

static void flyControllerTask(void* parameters) {
    while(true) {
        flyState_t state;
        if (pdTRUE == xQueueReceive(inputQueue, &state, portMAX_DELAY)) {
            // Call control on state
            // Set initial state
            flyController.output.ID = state.ID;
            state = filter_state(&flyController, state);
            if(counter < 500) {
                initPoint.X = (initPoint.X*counter + state.positionX) / (counter + 1);
                initPoint.Y = (initPoint.Y*counter + state.positionY) / (counter + 1);
                initPoint.Z = (initPoint.Z*counter + state.altitudeZ) / (counter + 1);
                ++counter;

                if(counter == 500) {
                    setPoint.X = initPoint.X;
                    setPoint.Y = initPoint.Y;
                    setPoint.Z = initPoint.Z;
                }
            }

            else {
                ++counter;
                if(counter == 2500) {
                    setPoint.Z = setPoint.Z + 0.2f;
                }
                control(&flyController, state, setPoint);
                /* Enqueue control */
                xQueueSend(spiTaskQueueHandle, &(flyController.output), 0);
            }
        }
    }
}

void flyControllerTaskEnqueueInput(flyState_t state) {
    xQueueOverwrite(inputQueue, &state);
}

