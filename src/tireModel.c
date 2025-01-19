// C File: tireMode.c
#include "tireModel.h"
#include <math.h>

void    calculateTireForces(const TireParameters *param, const TireInputs *inputs, TireOutputs *output)
{
    //Peak Force
    float peakForce         = param->mu * inputs->normalForce;

    //Lateral Force (Fy)
    output->lateralForce    = peakForce * sin(param->shapeFactor * atan(param->stiffnessFactor * inputs->slipAngle));
    output->longForce       = 0;
}