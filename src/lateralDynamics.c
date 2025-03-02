#include "lateralDynamics.h"
#include <math.h>

//Function to calculate loads on the wheels
void calculateWheelLoads(LateralDynamics *latDyn)
{
    //Calculate load transfer
    float latLoadTransferFront      = g_vehicleParam.mass * g_vehicleParam.weightBiasFront * latDyn->lateralAcceleration * g_vehicleParam.cgHeight / g_vehicleParam.wheelbase;
    float latLoadTransferRear       = g_vehicleParam.mass * (1 - g_vehicleParam.weightBiasFront) * latDyn->lateralAcceleration * g_vehicleParam.cgHeight / g_vehicleParam.wheelbase;

    //Calculate normal forces acting on the tires
    latDyn->normalForceFrontLeft   = (g_vehicleParam.mass * 9.81 * g_vehicleParam.weightBiasFront / 2) - latLoadTransferFront;
    latDyn->normalForceFrontRight   = (g_vehicleParam.mass * 9.81 * g_vehicleParam.weightBiasFront / 2) + latLoadTransferFront;
    latDyn->normalForceRearLeft    = (g_vehicleParam.mass * 9.81 * (1 - g_vehicleParam.weightBiasFront) / 2) - latLoadTransferRear;
    latDyn->normalForceRearRight    = (g_vehicleParam.mass * 9.81 * (1 - g_vehicleParam.weightBiasFront) / 2) + latLoadTransferRear;
}

//Calculate slip angles of the tires
void calculateSlipAngles(LongitudinalDynamics *longDyn, LateralDynamics *latDyn, const DrivingCommands *drivingCmd)
{
    latDyn->slipAngleFrontLeft     = atan((latDyn->lateralVelocity + (1-g_vehicleParam.weightBiasFront) * g_vehicleParam.wheelbase * latDyn->yawRate)/(longDyn->longitudinalVelocity - latDyn->yawRate*g_vehicleParam.trackWidthFront/2)) - drivingCmd->steeringAngle;
    latDyn->slipAngleFrontRight     = atan((latDyn->lateralVelocity + (1-g_vehicleParam.weightBiasFront) * g_vehicleParam.wheelbase * latDyn->yawRate)/(longDyn->longitudinalVelocity + latDyn->yawRate*g_vehicleParam.trackWidthFront/2)) - drivingCmd->steeringAngle;
    latDyn->slipAngleRearLeft      = atan((latDyn->lateralVelocity - g_vehicleParam.weightBiasFront * g_vehicleParam.wheelbase * latDyn->yawRate)/(longDyn->longitudinalVelocity - latDyn->yawRate*g_vehicleParam.trackWidthRear/2));
    latDyn->slipAngleRearRight      = atan((latDyn->lateralVelocity - g_vehicleParam.weightBiasFront * g_vehicleParam.wheelbase * latDyn->yawRate)/(longDyn->longitudinalVelocity + latDyn->yawRate*g_vehicleParam.trackWidthRear/2));
}


//Function to calculate lateral dynamics
void calculateLateralDynamics(const TireOutputs *tireOutput, LateralDynamics *latDyn, LongitudinalDynamics *longDyn)
{
    float frontLateralForce = tireOutput->lateralForceFrontLeft + tireOutput->lateralForceFrontRight;
    float rearLateralForce = tireOutput->lateralForceRearLeft + tireOutput->lateralForceRearRight;

    float totalLateralForce = tireOutput->lateralForceFrontLeft + tireOutput->lateralForceFrontRight + tireOutput->lateralForceRearLeft + tireOutput->lateralForceRearRight;

    //Calculate yaw acceleration and body slip rate
    float yawAcceleration = ((frontLateralForce * (1-g_vehicleParam.weightBiasFront)*g_vehicleParam.wheelbase) - (rearLateralForce * g_vehicleParam.weightBiasFront * g_vehicleParam.wheelbase))/g_vehicleParam.momentOfInertia;

    //Calculate yaw acceleration
    latDyn->yawRate = latDyn->yawRate + yawAcceleration * g_simulationParam.timeStep;

    //Calculate lateral acceleration
    latDyn->lateralAcceleration = totalLateralForce/g_vehicleParam.mass;

    //Calculate lateral velocity
    float latVelocityDot = latDyn->lateralAcceleration - latDyn->yawRate*longDyn->longitudinalVelocity;
    latDyn->lateralVelocity = latDyn->lateralVelocity + latVelocityDot*g_simulationParam.timeStep;

    //Calculate body-slip
    latDyn->bodySlipAngle = atan(latDyn->lateralVelocity/longDyn->longitudinalVelocity);

}