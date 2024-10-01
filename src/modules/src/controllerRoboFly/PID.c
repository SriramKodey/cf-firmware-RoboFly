#include "PID.h"

void PID_Init(PID_t *PID, float P, float I, float D, float dt)
{
    PID->P = P;
    PID->I = I;
    PID->D = D;
    PID->dt = dt;
    PID->lastError = 0;
    PID->errorSum = 0;
    PID->firstRun = true;
}

float calc_PID_Output(PID_t *PID, float actual, float setpoint)
{
    float error = setpoint - actual;
    float Pout = PID->P*(error);
    if(PID->firstRun){
        PID->lastError = error;
        PID->firstRun = false;
    }
    float Dout = PID->D * (error - PID->lastError)/PID->dt;
    PID->lastError = error;
    float Iout = PID->I * (PID->errorSum);
    PID->errorSum += error * PID->dt;
    return(Pout + Dout + Iout);
}