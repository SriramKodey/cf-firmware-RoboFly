#include "config.h"
#include "debug.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "static_mem.h"
#include "task.h"

#include "deck_spi.h"
#include "deck.h"
#include "stm32f4xx_spi.h"
#include "deck_constants.h"

#include "disc_spi.h"
#include "flyController_PID.h"

static xQueueHandle inputQueue;
STATIC_MEM_QUEUE_ALLOC(inputQueue, 5, sizeof(flyControl_t));

static void discSpiTask(void *);
STATIC_MEM_TASK_ALLOC(discSpiTask, DISC_SPI_TASK_STACKSIZE);

static deckPin_t cs_Pin;

static uint16_t spiSpeed = SPI_BAUDRATE_21MHZ;
static uint8_t spiTxBuffer[20];
static uint8_t spiRxBuffer[10];
static bool isInit = false;

static discPacket_t writePacket;

xQueueHandle discSpiTaskInit() {
    inputQueue = STATIC_MEM_QUEUE_CREATE(inputQueue);

    /* Setup SPI */
    // Init Pins
    cs_Pin = DECK_GPIO_IO3;
    pinMode(cs_Pin, OUTPUT);
    digitalWrite(cs_Pin, HIGH);
    spiBegin();

    STATIC_MEM_TASK_CREATE(discSpiTask, discSpiTask, DISC_SPI_TASK_NAME, NULL, DISC_SPI_TASK_PRI);
    isInit = true;

    return inputQueue;
}

bool discSpiTaskTest() {
    return isInit;
}

static void discSpiTask(void * parameters) {
    DEBUG_PRINT("DISC_SPI_TASK main function is running");
    writePacket.amplitude = 150.0;
    writePacket.delta_amplitude = 20.0;
    writePacket.offset = 5.0;
    flyControl_t input;
    while(true) {
        if (pdTRUE == xQueueReceive(inputQueue, &input, portMAX_DELAY)) {
            // set current data packet values
            writePacket.amplitude = input.amplitude;
            writePacket.delta_amplitude = input.delta_amplitude;
            writePacket.offset = input.offset;
        }
        /* Control module will initiate transmission */
        spiBeginTransaction(spiSpeed);
        memcpy(spiTxBuffer, &writePacket, 12);
        digitalWrite(cs_Pin, LOW);
        spiExchange(sizeof(discPacket_t), spiTxBuffer, spiRxBuffer);
        digitalWrite(cs_Pin, HIGH);
        spiEndTransaction();
    }
}

void discSpiTaskEnqueueInput(float value) {
    xQueueOverwrite(inputQueue, &value);
}