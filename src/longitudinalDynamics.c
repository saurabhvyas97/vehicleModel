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