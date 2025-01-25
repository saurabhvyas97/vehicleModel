#include "longitudinalDynamics.h"
#include "lateralDynamics.h"
#include "drivingCommands.h"
#include <math.h>

//Function to calculate loads on the wheels
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

//Calculate slip angles of the tires
void calculateSlipAngles(const LongitudinalDynamics *longDyn, LateralDynamics *latDyn, const DrivingCommands *drivingCmd)
{
    latDyn->slipAngleFrontInner     = atan((latDyn->lateralVelocity + (1-g_vehicleParam.weightBiasFront) * g_vehicleParam.wheelbase * latDyn->yawRate)/(longDyn->longitudinalVelocity - latDyn->yawRate*g_vehicleParam.trackWidthFront/2)) - drivingCmd->steeringAngle;
    latDyn->slipAngleFrontOuter     = atan((latDyn->lateralVelocity + (1-g_vehicleParam.weightBiasFront) * g_vehicleParam.wheelbase * latDyn->yawRate)/(longDyn->longitudinalVelocity + latDyn->yawRate*g_vehicleParam.trackWidthFront/2)) - drivingCmd->steeringAngle;
    latDyn->slipAngleRearInner      = atan((latDyn->lateralVelocity - g_vehicleParam.weightBiasFront * g_vehicleParam.wheelbase * latDyn->yawRate)/(longDyn->longitudinalVelocity - latDyn->yawRate*g_vehicleParam.trackWidthRear/2));
    latDyn->slipAngleRearOuter      = atan((latDyn->lateralVelocity - g_vehicleParam.weightBiasFront * g_vehicleParam.wheelbase * latDyn->yawRate)/(longDyn->longitudinalVelocity + latDyn->yawRate*g_vehicleParam.trackWidthRear/2));
}