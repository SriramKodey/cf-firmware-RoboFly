#ifndef __DISC_SPI_H__
#define __DISC_SPI_H__

#include <stdbool.h>
#include <queue.h>

#include "flyController_PID.h"

typedef struct __attribute__((__packed__)) {
    float offset;
    float amplitude;
    float delta_amplitude;
    float delta_offset;
    uint32_t ID;
} discPacket_t;

QueueHandle_t discSpiTaskInit();
bool discSpiTaskTest();

void discSpiTaskEnqueueInput(flyControl_t value);

#endif