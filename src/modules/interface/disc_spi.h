#ifndef __DISC_SPI_H__
#define __DISC_SPI_H__

#include <stdbool.h>

typedef struct __attribute__((__packed__)){
    float amplitude;
    float delta_amplitude;
    float offset;
}discPacket_t;

void discSpiTaskInit();
bool discSpiTaskTest();

void discSpiTaskEnqueueInput(int value);

#endif