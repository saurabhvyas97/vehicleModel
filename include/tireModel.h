// tireModel.h
#ifndef TIRE_MODEL_H
#define TIRE_MODEL_H

#include "simulationTypes.h"

// Function for calculating tire forces
void calculateTireForces(const TireParameters *param, const TireInputs *inputs, TireOutputs *outputs);

//Function to calculate tire forces for individual wheels of the Vehicle
void calculateVehicleTireForces(const TireParameters *param, LateralDynamics *latDyn, TireInputs *inputs, TireOutputs *tireOutput);

#endif