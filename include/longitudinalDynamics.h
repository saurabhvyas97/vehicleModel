// Header file for Longitudinal Dynamics: longitudinalDynamics.h

#ifndef LONGITUDINAL_DYNAMICS_H
#define LONGITUDINAL_DYNAMICS_H

#include "simulationTypes.h"

// Function to calculate loads on the wheels resulting from longitudinal load transfer
void calculateWheelLoads_LongLT(LongitudinalDynamics *longDyn, TireInputs *tireInput);

#endif