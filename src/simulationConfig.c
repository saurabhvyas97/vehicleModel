#include "simulationConfig.h"

SimulationParameters g_simulationParam = {
    .timeStep           = 0.001,
    .endTime            = 50,
    .initialVelocity    = 0,
    .initialYawRate     = 0,
    .initialWheelAngle  = 0,
    .rad2deg            = 57.3,
    .deg2rad            = 1/57.3,
    .gravity            = 9.81
};