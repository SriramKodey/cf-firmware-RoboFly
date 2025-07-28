#ifndef __MOCAP_UART_H__
#define __MOCAP_UART_H__

#include <stdbool.h>
#include "queue.h"

typedef struct {
    QueueHandle_t stateMachineTaskQueueHandle;
    QueueHandle_t flyControllerTaskQueueHandle;
} mocapTaskQueueHandleInput_t;

void mocapTaskInit(mocapTaskQueueHandleInput_t mocapTaskQueueHandleInput);
bool mocapTaskTest();

void mocapTaskEnqueueInput(int value);

#endif

