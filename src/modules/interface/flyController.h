#pragma once

#include <stdbool.h>

#include "flyController_PID.h" 

void flyControllerTaskInit();
bool flyControllerTaskTest();

void flyControllerTaskEnqueueInput(flyState_t state);