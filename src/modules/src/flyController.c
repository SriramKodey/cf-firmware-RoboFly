#include "config.h"
#include "debug.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "static_mem.h"
#include "task.h"

#include "flyController.h"
#include "disc_spi.h"

/* File Static Varirables */
static xQueueHandle inputQueue;
STATIC_MEM_QUEUE_ALLOC(inputQueue, 5, sizeof(flyState_t));

/* flyController struct - needs to be static!! */
static flyController_PID_t flyController;
static desriedPosition_t setPoint;
static QueueHandle_t spiTaskQueueHandle;

/* Task prototype and Stack allocation */
static void flyControllerTask(void *);
STATIC_MEM_TASK_ALLOC(flyControllerTask, CONTROLLER_TASK_STACKSIZE);

static bool isInit = false;

void flyControllerTaskInit(QueueHandle_t sendQueue) {
    inputQueue = STATIC_MEM_QUEUE_CREATE(inputQueue);

    spiTaskQueueHandle = sendQueue;
    /* Initialise flyController */
    flyController_PID_Init(&flyController);
    setPoint.X = 0.5;
    setPoint.Y = 0.5;
    setPoint.Z = 0.2;

    STATIC_MEM_TASK_CREATE(flyControllerTask, flyControllerTask, CONTROLLER_TASK_NAME, NULL, CONTROLLER_TASK_PRI);
    isInit = true;
}

bool flyControllerTaskTest() {
    return isInit;
}

static void flyControllerTask(void* parameters) {
    while(true) {
        flyState_t state;
        if (pdTRUE == xQueueReceive(inputQueue, &state, 0)) {
            // Call control on state
        }

        // Set sample state
        state.altitudeZ = 0;
        state.positionX = 0;
        state.positionY = 0;
        state.quat_w = 1;
        state.quat_i = 0;
        state.quat_j = 0;
        state.quat_k = 0;

        flyController.state = state;
        // Give state to control 
        control(&flyController, state, setPoint);

        /* Enqueue control */
        xQueueSend(spiTaskQueueHandle, &(flyController.output.amplitude), 0);

        /*** DELAY ***/
        // Added during Dev, edit later //
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void flyControllerTaskEnqueueInput(flyState_t state) {
    xQueueOverwrite(inputQueue, &state);
}