#include "config.h"
#include "debug.h"
#include "FreeRTOS.h"
#include "static_mem.h"
#include "task.h"
#include "uart2.h"
#include "led.h"

#include "mocap_uart.h"
#include "flyController.h"
#include "flyController_PID.h"
#include "system.h"

static xQueueHandle inputQueue;
STATIC_MEM_QUEUE_ALLOC(inputQueue, 5, sizeof(int));

static QueueHandle_t flyControllerTaskQueueHandle;
static QueueHandle_t stateMachineTaskQueueHandle;
static TickType_t xLastWakeTime;

static TickType_t lastGoodPacketTime = 0;
static TickType_t lastLEDToggleTime = 0;

static const uint32_t uartTimeout = 2; // in ms

static bool ledOn = false;

static void mocapTask(void*);
STATIC_MEM_TASK_ALLOC(mocapTask, MOCAP_UART_TASK_STACKSIZE);

static flyState_t stateEstimate;
static bool isInit = false;

static const uint32_t BAUD_RATE = (uint32_t) 230400;
static const int bufferSize = 33;

void mocapTaskInit(mocapTaskQueueHandleInput_t sendQueues) {
    inputQueue = STATIC_MEM_QUEUE_CREATE(inputQueue);

    flyControllerTaskQueueHandle = sendQueues.flyControllerTaskQueueHandle;
    stateMachineTaskQueueHandle = sendQueues.stateMachineTaskQueueHandle;

    uart2Init(BAUD_RATE);

    STATIC_MEM_TASK_CREATE(mocapTask, mocapTask, MOCAP_UART_TASK_NAME, NULL, MOCAP_UART_TASK_PRI);
    isInit = true;
}

bool mocapTaskTest() {
    return isInit;
}

static void mocapTask(void * parameters) {
    systemWaitStart();
    xLastWakeTime = xTaskGetTickCount();
    while (true) {
        char startByte = 'i';
        char testByte;
        uint8_t readBuffer[50];
        bool goodRead = false;

        do {
            uart2GetDataWithTimeout(1, &startByte, uartTimeout);
            xLastWakeTime = xTaskGetTickCount();
        } while(startByte != 's' && startByte != 'e');

        // because it's only the start trigger its ok to not be instanteous
        xQueueSend(stateMachineTaskQueueHandle, &startByte, pdMS_TO_TICKS(0));

        if (startByte == 's') {
            if (uart2GetData(bufferSize, readBuffer) >= 32) {
                memcpy(&testByte, &(readBuffer[32]), 1);
                if (testByte == '\n') {
                    goodRead = true;
                    lastGoodPacketTime = xTaskGetTickCount();

                    // Check mapping later
                    memcpy(&(stateEstimate.ID), &readBuffer, 4);
                    memcpy(&(stateEstimate.positionX), &(readBuffer[4]), 4);
                    memcpy(&(stateEstimate.positionY), &(readBuffer[8]), 4);
                    memcpy(&(stateEstimate.altitudeZ), &(readBuffer[12]), 4);
                    memcpy(&(stateEstimate.quat_w), &(readBuffer[16]), 4);
                    memcpy(&(stateEstimate.quat_i), &(readBuffer[20]), 4);
                    memcpy(&(stateEstimate.quat_j), &(readBuffer[24]), 4);
                    memcpy(&(stateEstimate.quat_k), &(readBuffer[28]), 4);

                    xQueueSend(flyControllerTaskQueueHandle, &stateEstimate, pdMS_TO_TICKS(0));
                }
            }
        }

        else if (startByte == 'e') {
            // ramp down logic in state machine
            xQueueSend(flyControllerTaskQueueHandle, &stateEstimate, pdMS_TO_TICKS(0));
        }
        
        TickType_t now = xTaskGetTickCount();
        // Toggle LED every 100 ms if data is coming in
        if ((now - lastLEDToggleTime) >= pdMS_TO_TICKS(200)) {
            if ((now - lastGoodPacketTime) < pdMS_TO_TICKS(100)) {
                ledOn = !ledOn;
                ledSet(LED_GREEN_L, ledOn);  // Manual toggle
            }
            lastLEDToggleTime = now;
        }

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(4));
    }
}

void mocapTaskEnqueueInput(int value) {
    xQueueOverwrite(inputQueue, &value);
}