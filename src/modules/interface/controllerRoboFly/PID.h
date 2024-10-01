#ifndef __PID_H__
#define __PID_H__

#include "stdbool.h"

typedef struct{
    float P;
    float I;
    float D;
    float lastError;
    float lastOutput;
    float errorSum;
    float dt;
    bool firstRun;
}PID_t;

void PID_Init(PID_t *PID, float P, float I, float D, float dt);

float calc_PID_Output(PID_t *PID, float actual, float setpoint);

#endif