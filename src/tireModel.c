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

void    calculateVehicleTireForces(const TireParameters *param, LateralDynamics *latDyn, TireInputs *inputs, TireOutputs *tireOutput)
{
    float normalLoadFrontLeft = -latDyn->normalForceFrontLeft;
    float normalLoadFrontRight = -latDyn->normalForceFrontRight;
    float normalLoadRearLeft  = -latDyn->normalForceRearLeft;
    float normalLoadRearRight  = -latDyn->normalForceRearRight;


    //Lateral Force front Left (Fy)
    inputs->normalForce = normalLoadFrontLeft;
    inputs->slipAngle   = 57.3*latDyn->slipAngleFrontLeft;
    calculateTireForces(param, inputs, tireOutput);
    tireOutput->lateralForceFrontLeft = tireOutput->lateralForce;

    //Lateral Force front Right (Fy)
    inputs->normalForce = normalLoadFrontRight;
    inputs->slipAngle   = 57.3*latDyn->slipAngleFrontRight;
    calculateTireForces(param, inputs, tireOutput);
    tireOutput->lateralForceFrontRight = tireOutput->lateralForce;

    //Lateral Force rear Left (Fy)
    inputs->normalForce = normalLoadRearLeft;
    inputs->slipAngle   = 57.3*latDyn->slipAngleRearLeft;
    calculateTireForces(param, inputs, tireOutput);
    tireOutput->lateralForceRearLeft = tireOutput->lateralForce;

    //Lateral Force rear Right (Fy)
    inputs->normalForce = normalLoadRearRight;
    inputs->slipAngle   = 57.3*latDyn->slipAngleRearRight;
    calculateTireForces(param, inputs, tireOutput);
    tireOutput->lateralForceRearRight = tireOutput->lateralForce;
}

