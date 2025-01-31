// C File: tireMode.c
#include "tireModel.h"
#include "lateralDynamics.h"
#include <math.h>

void    calculateTireForces(const TireParameters *param, const TireInputs *inputs, TireOutputs *output)
{
    //Peak Force
    float normalizedForce         = (param->mu + param->stiffnessFactor/1000*inputs->normalForce)*inputs->normalForce;

    //Lateral Force (Fy)
    output->lateralForce    = normalizedForce * sin(param->peakForce * atan(param->shapeFactor * inputs->slipAngle));
    output->longForce       = 0;
}

void    calculateVehicleTireForces(const TireParameters *param, LateralDynamics *latDyn, TireInputs *inputs, TireOutputs *tireOutput)
{
    float normalLoadFrontInner = -latDyn->normalForceFrontInner;
    float normalLoadFrontOuter = -latDyn->normalForceFrontOuter;
    float normalLoadRearInner  = -latDyn->normalForceRearInner;
    float normalLoadRearOuter  = -latDyn->normalForceRearOuter;


    //Lateral Force front inner (Fy)
    inputs->normalForce = normalLoadFrontInner;
    inputs->slipAngle   = 57.3*latDyn->slipAngleFrontInner;
    calculateTireForces(param, inputs, tireOutput);
    tireOutput->lateralForceFrontInner = tireOutput->lateralForce;

    //Lateral Force front outer (Fy)
    inputs->normalForce = normalLoadFrontOuter;
    inputs->slipAngle   = 57.3*latDyn->slipAngleFrontOuter;
    calculateTireForces(param, inputs, tireOutput);
    tireOutput->lateralForceFrontOuter = tireOutput->lateralForce;

    //Lateral Force rear inner (Fy)
    inputs->normalForce = normalLoadRearInner;
    inputs->slipAngle   = 57.3*latDyn->slipAngleRearInner;
    calculateTireForces(param, inputs, tireOutput);
    tireOutput->lateralForceRearInner = tireOutput->lateralForce;

    //Lateral Force rear outer (Fy)
    inputs->normalForce = normalLoadRearOuter;
    inputs->slipAngle   = 57.3*latDyn->slipAngleRearOuter;
    calculateTireForces(param, inputs, tireOutput);
    tireOutput->lateralForceRearOuter = tireOutput->lateralForce;
}

