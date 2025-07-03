#include "config.h"
#include "debug.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "static_mem.h"
#include "task.h"
#include "led.h"

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

static TickType_t xLastWakeTime;
static deckPin_t cs_Pin;

static TickType_t lastLEDToggleTime = 0;
static bool ledOn = false;

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
    writePacket.amplitude = 0.0;
    writePacket.delta_amplitude = 0.0;
    writePacket.offset = 0.0;
    writePacket.mu = 0.0;
    writePacket.ID = 0;
    xLastWakeTime = xTaskGetTickCount();
    flyControl_t input;
    while(true) {
        if (pdTRUE == xQueueReceive(inputQueue, &input, portMAX_DELAY)) {
            // set current data packet values
            writePacket.ID = input.ID;
            writePacket.amplitude = input.amplitude;
            writePacket.delta_amplitude = input.delta_amplitude;
            writePacket.offset = input.offset;
            writePacket.mu = writePacket.mu + 1;
            writePacket.tickTime = (uint32_t) xTaskGetTickCount();
        }
        /* Control module will initiate transmission */
        spiBeginTransaction(spiSpeed);
        memcpy(spiTxBuffer, &writePacket, 24);
        digitalWrite(cs_Pin, LOW);
        spiExchange(sizeof(discPacket_t), spiTxBuffer, spiRxBuffer);
        digitalWrite(cs_Pin, HIGH);
        spiEndTransaction();
        TickType_t now = xTaskGetTickCount();
        // Toggle LED every 100 ms if data is coming in
        if ((now - lastLEDToggleTime) >= pdMS_TO_TICKS(200)) {
            ledOn = !ledOn;
            ledSet(LED_BLUE_L, ledOn);  // Manual toggle
            lastLEDToggleTime = now;
        }
        // vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(4));
    }
}

void discSpiTaskEnqueueInput(float value) {
    xQueueOverwrite(inputQueue, &value);
}