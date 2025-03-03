// lateralDynamics.h
#ifndef LATERAL_DYNAMICS_H
#define LATERAL_DYNAMICS_H

#include "simulationTypes.h"


// Function to calculate loads on the wheels resulting from lateral load transfer
void calculateWheelLoads_LatLT(LateralDynamics *latDyn);

// Calculate slip angles of the tires
void calculateSlipAngles(LongitudinalDynamics *longDyn, LateralDynamics *latDyn, const DrivingCommands *drivingCmd);

// Function to calculate lateral dynamics
void calculateLateralDynamics(const TireOutputs *tireOutput, LateralDynamics *latDyn, LongitudinalDynamics *longDyn);

#endif