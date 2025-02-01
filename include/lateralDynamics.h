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
    float slipAngleFrontLeft;         // Slip angle of the front Left tires in rad
    float slipAngleFrontRight;         // Slip angle of the front Right tires in rad
    float slipAngleRearLeft;          // Slip angle of the rear Left tires in rad
    float slipAngleRearRight;         // Slip angle of the rear Right tires in rad
    float normalForceFrontLeft;      // Normal force acting on the front Left tires in N
    float normalForceFrontRight;      // Normal force acting on the front Right tires in N
    float normalForceRearLeft;       // Normal force acting on the rear Left tires in N
    float normalForceRearRight;       // Normal force acting on the rear Right tires in N
    float totalLateralForce;          // Total lateral force acting on the vehicle in N
} LateralDynamics;

// Function to calculate loads on the wheels
void calculateWheelLoads(LateralDynamics *latDyn);

// Calculate slip angles of the tires
void calculateSlipAngles(LongitudinalDynamics *longDyn, LateralDynamics *latDyn, const DrivingCommands *drivingCmd);

// Function to calculate lateral dynamics
void calculateLateralDynamics(const TireOutputs *tireOutput, LateralDynamics *latDyn, LongitudinalDynamics *longDyn);

#endif