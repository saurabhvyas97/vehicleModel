// lateralDynamics.h
#ifndef LATERAL_DYNAMICS_H
#define LATERAL_DYNAMICS_H

#include "vehicleConfig.h"
#include "simulationConfig.h"
#include "longitudinalDynamics.h"
#include "drivingCommands.h"
#include "tireModel.h"

// Define structure for lateral dynamics outputs
typedef struct LateralDynamics {
    float lateralAcceleration;        // Lateral acceleration of the vehicle in m/s^2
    float lateralVelocity;            // Lateral velocity of the vehicle in m/s
    float bodySlipAngle;              // Body slip angle of the vehicle in rad
    float yawRate;                    // Yaw rate of the vehicle in rad/s
    float bodyRollAngle;              // Body roll angle of the vehicle in rad
    float slipAngleFrontInner;         // Slip angle of the front inner tires in rad
    float slipAngleFrontOuter;         // Slip angle of the front outer tires in rad
    float slipAngleRearInner;          // Slip angle of the rear inner tires in rad
    float slipAngleRearOuter;         // Slip angle of the rear outer tires in rad
    float normalForceFrontInner;      // Normal force acting on the front inner tires in N
    float normalForceFrontOuter;      // Normal force acting on the front outer tires in N
    float normalForceRearInner;       // Normal force acting on the rear inner tires in N
    float normalForceRearOuter;       // Normal force acting on the rear outer tires in N
    float totalLateralForce;          // Total lateral force acting on the vehicle in N
} LateralDynamics;

// Function to calculate loads on the wheels
void calculateWheelLoads(LateralDynamics *latDyn);

// Calculate slip angles of the tires
void calculateSlipAngles(LongitudinalDynamics *longDyn, LateralDynamics *latDyn, const DrivingCommands *drivingCmd);

// Function to calculate lateral dynamics
void calculateLateralDynamics(const TireOutputs *tireOutput, LateralDynamics *latDyn, LongitudinalDynamics *longDyn);

#endif