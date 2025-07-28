#pragma once

#include <stdbool.h>

#include "queue.h"
#include "task.h"
#include "flyController_PID.h" 

typedef struct {    
    TaskHandle_t flyControllerTaskHandle;
    QueueHandle_t flyControllerQueueHandle;
} flyControllerHandles_t;

typedef enum {
    MODE_IDLE,
    MODE_LIFTOFF,
    MODE_CONTROL_ON,
    MODE_LAND,
    MODE_RESET,
    MODE_NONE
} ControllerMode_t;

flyControllerHandles_t flyControllerTaskInit(QueueHandle_t sendQueue);

bool flyControllerTaskTest();

void flyControllerTaskEnqueueInput(flyState_t state);