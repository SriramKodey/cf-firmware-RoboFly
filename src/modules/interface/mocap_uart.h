#ifndef __MOCAP_UART_H__
#define __MOCAP_UART_H__

#include <stdbool.h>
#include "queue.h"

void mocapTaskInit(QueueHandle_t sendQueue);
bool mocapTaskTest();

void mocapTaskEnqueueInput(int value);

#endif

