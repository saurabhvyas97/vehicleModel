// C File: tireMode.c
#include "tireModel.h"
#include <math.h>

void    calculateTireForces(const TireParameters *param, const TireInputs *inputs, TireOutputs *output)
{
    //Peak Force
    float normalizedForce         = (param->mu + param->stiffnessFactor/1000*inputs->normalForce)*inputs->normalForce;

    //Lateral Force (Fy)
    output->lateralForce    = normalizedForce * sin(param->peakForce * atan(param->shapeFactor * inputs->slipAngle));
    output->longForce       = 0;
}