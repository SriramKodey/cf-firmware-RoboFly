#include "config.h"
#include "debug.h"
#include "FreeRTOS.h"
#include "static_mem.h"
#include "task.h"
#include "uart2.h"

#include "flyController.h"
#include "flyController_PID.h"

static xQueueHandle inputQueue;
STATIC_MEM_QUEUE_ALLOC(inputQueue, 1, sizeof(int));

static QueueHandle_t flyControllerTaskQueueHandle;

static void mocapTask(void*);
STATIC_MEM_TASK_ALLOC(mocapTask, MOCAP_UART_TASK_STACKSIZE);

static bool isInit = false;

static const uint32_t BAUD_RATE = (uint32_t) 115200;
static const int bufferSize = 33;

void mocapTaskInit(QueueHandle_t sendQueue) {
    inputQueue = STATIC_MEM_QUEUE_CREATE(inputQueue);

    flyControllerTaskQueueHandle = sendQueue;

    uart2Init(BAUD_RATE);

    STATIC_MEM_TASK_CREATE(mocapTask, mocapTask, MOCAP_UART_TASK_NAME, NULL, MOCAP_UART_TASK_PRI);
    isInit = true;
}

bool mocapTaskTest() {
    return isInit;
}

static void mocapTask(void* parameters) {
    DEBUG_PRINT("MocapTASK main function is running!");
    while (true) {
        int input;
        char testByte;
        uint8_t readBuffer[50];
        bool readStart = false;
        bool goodRead = false;

        if (pdTRUE == xQueueReceive(inputQueue, &input, 0)) {
            // Write the data read, conversion to floats and send to flyController Queue

            // Read a byte and check if startByte
            uart2GetData(1, readBuffer);
            memcpy(&testByte, readBuffer, 1);

            if (testByte == 's') {
                readStart = true;
                if (uart2GetData(bufferSize, readBuffer) >= 33) {
                    memcpy(&testByte, &(readBuffer[32]), 1);
                    if (testByte == '\n') {
                        goodRead = true;
                        flyState_t stateEstimate;

                        // Check mapping later
                        memcpy(&(stateEstimate.positionX), &(readBuffer[4]), 4);
                        memcpy(&(stateEstimate.positionY), &(readBuffer[8]), 4);
                        memcpy(&(stateEstimate.altitudeZ), &(readBuffer[12]), 4);
                        memcpy(&(stateEstimate.quat_w), &(readBuffer[16]), 4);
                        memcpy(&(stateEstimate.quat_i), &(readBuffer[20]), 4);
                        memcpy(&(stateEstimate.quat_j), &(readBuffer[24]), 4);
                        memcpy(&(stateEstimate.quat_k), &(readBuffer[28]), 4);

                        flyControllerTaskEnqueueInput(stateEstimate);
                    }
                }
            }

            if ((readStart == true) && (goodRead == false)) {
                flyState_t stateEstimate;
                stateEstimate.positionX = 150.0;
                flyControllerTaskEnqueueInput(stateEstimate);
            }
        }
    }
}

void mocapTaskEnqueueInput(int value) {
    xQueueOverwrite(inputQueue, &value);
}