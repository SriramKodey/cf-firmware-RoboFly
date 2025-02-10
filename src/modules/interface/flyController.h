#pragma once

#include <stdbool.h>

#include "queue.h"

#include "flyController_PID.h" 

xQueueHandle flyControllerTaskInit(QueueHandle_t sendQueue);
bool flyControllerTaskTest();

void flyControllerTaskEnqueueInput(flyState_t state);