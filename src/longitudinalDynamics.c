#include "longitudinalDynamics.h"
#include <math.h>

//Function to calculate loads on the wheels resulting from longitudinal load transfer
void calculateWheelLoads_LongLT(LongitudinalDynamics *longDyn, TireInputs *tireInput)
{
    //Calculate load transfer
    float longLoadTransfer      = g_vehicleParam.mass * longDyn->longitudinalAcceleration * g_vehicleParam.cgHeight / g_vehicleParam.wheelbase;
    
    //Calculate normal forces acting on the tires
    tireInput->normalForceFrontLeft     = (g_vehicleParam.mass * 9.81 * g_vehicleParam.weightBiasFront / 2) - longLoadTransfer;
    tireInput->normalForceFrontRight    = (g_vehicleParam.mass * 9.81 * g_vehicleParam.weightBiasFront / 2) + longLoadTransfer;
    tireInput->normalForceRearLeft      = (g_vehicleParam.mass * 9.81 * (1 - g_vehicleParam.weightBiasFront) / 2) - longLoadTransfer;
    tireInput->normalForceRearRight     = (g_vehicleParam.mass * 9.81 * (1 - g_vehicleParam.weightBiasFront) / 2) + longLoadTransfer;
}

// Function to calculate slip ratios of the tires
void calculateSlipRatios(LongitudinalDynamics *longDyn, TireInputs *tireInputs, WheelDynamics *wheelDyn)
{
    if (longDyn->longitudinalVelocity<wheelDyn->wheelAngularSpeedFrontLeft*g_vehicleParam.wheelRadius)
    {
        tireInputs->slipRatioFrontLeft = (wheelDyn->wheelAngularSpeedFrontLeft*g_vehicleParam.wheelRadius - longDyn->longitudinalVelocity) / wheelDyn->wheelAngularSpeedFrontLeft*g_vehicleParam.wheelRadius;
    }
    else
    {
        tireInputs->slipRatioFrontLeft = (longDyn->longitudinalVelocity - wheelDyn->wheelAngularSpeedFrontLeft*g_vehicleParam.wheelRadius) / longDyn->longitudinalVelocity;
    }
    
    if (longDyn->longitudinalVelocity<wheelDyn->wheelAngularSpeedFrontRight*g_vehicleParam.wheelRadius)
    {
        tireInputs->slipRatioFrontRight = (wheelDyn->wheelAngularSpeedFrontRight*g_vehicleParam.wheelRadius - longDyn->longitudinalVelocity) / wheelDyn->wheelAngularSpeedFrontRight*g_vehicleParam.wheelRadius;
    }
    else
    {
        tireInputs->slipRatioFrontRight = (longDyn->longitudinalVelocity - wheelDyn->wheelAngularSpeedFrontRight*g_vehicleParam.wheelRadius) / longDyn->longitudinalVelocity;
    }

    if (longDyn->longitudinalVelocity<wheelDyn->wheelAngularSpeedRearLeft*g_vehicleParam.wheelRadius)
    {
        tireInputs->slipRatioRearLeft = (wheelDyn->wheelAngularSpeedRearLeft*g_vehicleParam.wheelRadius - longDyn->longitudinalVelocity) / wheelDyn->wheelAngularSpeedRearLeft*g_vehicleParam.wheelRadius;
    }
    else
    {
        tireInputs->slipRatioRearLeft = (longDyn->longitudinalVelocity - wheelDyn->wheelAngularSpeedRearLeft*g_vehicleParam.wheelRadius) / longDyn->longitudinalVelocity;
    }

    if (longDyn->longitudinalVelocity<wheelDyn->wheelAngularSpeedRearRight*g_vehicleParam.wheelRadius)
    {
        tireInputs->slipRatioRearRight = (wheelDyn->wheelAngularSpeedRearRight*g_vehicleParam.wheelRadius - longDyn->longitudinalVelocity) / wheelDyn->wheelAngularSpeedRearRight*g_vehicleParam.wheelRadius;
    }
    else
    {
        tireInputs->slipRatioRearRight = (longDyn->longitudinalVelocity - wheelDyn->wheelAngularSpeedRearRight*g_vehicleParam.wheelRadius) / longDyn->longitudinalVelocity;
    }
}