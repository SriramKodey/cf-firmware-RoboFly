#include "PID.h"

void PID_Init(PID_t *PID, float P, float I, float D, float dt)
{
    PID->P = P;
    PID->I = I;
    PID->D = D;
    PID->dt = dt;
    PID->lastActual = 0;
    PID->errorSum = 0;
    PID->firstRun = true;
}

float calc_PID_Output(PID_t *PID, float actual, float setpoint)
{
    float error = setpoint - actual;
    float Pout = PID->P*(error);
    if(PID->firstRun){
        PID->lastActual = actual;
        PID->firstRun = false;
    }
    // Run the derivative term on the output rather than on the error
    float Dout = PID->D * (PID->lastActual - actual)/PID->dt;
    PID->lastActual = actual;
    PID->errorSum += error * PID->dt;
    float Iout = PID->I * (PID->errorSum);
    return(Pout + Dout + Iout);
}