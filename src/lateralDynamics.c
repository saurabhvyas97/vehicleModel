#include "lateralDynamics.h"
#include <math.h>

void calculateWheelLoads(LateralDynamics *latDyn)
{
    //Calculate load transfer
    float latLoadTransferFront      = g_vehicleParam.mass * g_vehicleParam.weightBiasFront * latDyn->lateralAcceleration * g_vehicleParam.cgHeight / g_vehicleParam.wheelbase;
    float latLoadTransferRear       = g_vehicleParam.mass * (1 - g_vehicleParam.weightBiasFront) * latDyn->lateralAcceleration * g_vehicleParam.cgHeight / g_vehicleParam.wheelbase;

    //Calculate normal forces acting on the tires
    latDyn->normalForceFrontInner   = (g_vehicleParam.mass * 9.81 * g_vehicleParam.weightBiasFront / 2) - latLoadTransferFront;
    latDyn->normalForceFrontOuter   = g_vehicleParam.mass * 9.81 * g_vehicleParam.weightBiasFront / 2 + latLoadTransferFront;
    latDyn->normalForceRearInner    = g_vehicleParam.mass * 9.81 * (1 - g_vehicleParam.weightBiasFront) / 2 - latLoadTransferRear;
    latDyn->normalForceRearOuter    = g_vehicleParam.mass * 9.81 * (1 - g_vehicleParam.weightBiasFront) / 2 + latLoadTransferRear;
}
