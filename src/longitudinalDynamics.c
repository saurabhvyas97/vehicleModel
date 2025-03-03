#include "longitudinalDynamics.h"
#include <math.h>

//Function to calculate loads on the wheels resulting from longitudinal load transfer
void calculateWheelLoads_LongLT(LongitudinalDynamics *longDyn, TireInputs *tireInput)
{
    //Calculate load transfer
    float latLoadTransferFront      = g_vehicleParam.mass * g_vehicleParam.weightBiasFront * longDyn->longitudinalAcceleration * g_vehicleParam.cgHeight / g_vehicleParam.wheelbase;
    float latLoadTransferRear       = g_vehicleParam.mass * (1 - g_vehicleParam.weightBiasFront) * longDyn->longitudinalAcceleration * g_vehicleParam.cgHeight / g_vehicleParam.wheelbase;

    //Calculate normal forces acting on the tires
    tireInput->normalForceFrontLeft   = (g_vehicleParam.mass * 9.81 * g_vehicleParam.weightBiasFront / 2) - latLoadTransferFront;
    tireInput->normalForceFrontRight   = (g_vehicleParam.mass * 9.81 * g_vehicleParam.weightBiasFront / 2) + latLoadTransferFront;
    tireInput->normalForceRearLeft    = (g_vehicleParam.mass * 9.81 * (1 - g_vehicleParam.weightBiasFront) / 2) - latLoadTransferRear;
    tireInput->normalForceRearRight    = (g_vehicleParam.mass * 9.81 * (1 - g_vehicleParam.weightBiasFront) / 2) + latLoadTransferRear;
    
}