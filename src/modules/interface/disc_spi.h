#ifndef __DISC_SPI_H__
#define __DISC_SPI_H__

#include <stdbool.h>

typedef struct __attribute__((__packed__)) {
    uint32_t ID;
    float amplitude;
    float delta_amplitude;
    float offset;
    float mu;
    uint32_t tickTime;
} discPacket_t;

xQueueHandle discSpiTaskInit();
bool discSpiTaskTest();

void discSpiTaskEnqueueInput(float value);

#endif