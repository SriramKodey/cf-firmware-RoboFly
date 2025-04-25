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
    setPoint.X = -0.5;
    setPoint.Y = -0.5;
    setPoint.Z = 0.8;

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
            control(&flyController, state, setPoint);

            /* Enqueue control */
            xQueueSend(spiTaskQueueHandle, &(flyController.output), 0);
        }
    }
}

void flyControllerTaskEnqueueInput(flyState_t state) {
    xQueueOverwrite(inputQueue, &state);
}

// /**
//  * Logging variables for the command and reference signals for the
//  * PID flyController
//  */
// LOG_GROUP_START(flyControl)
// /**
//  * @brief Thrust command
//  */
// LOG_ADD(LOG_FLOAT,  amplitude, &(flyController.output.amplitude))
// /**
//  * @brief Roll command
//  */
// LOG_ADD(LOG_FLOAT, delta_amplitude, &(flyController.output.delta_amplitude))
// /**
//  * @brief Pitch command
//  */
// LOG_ADD(LOG_FLOAT, offset, &(flyController.output.offset))
// LOG_GROUP_STOP(flyControl)