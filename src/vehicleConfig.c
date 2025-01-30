//C File: vehicleConfig.c
#include "vehicleConfig.h"

//Initalize vehicle parameters
VehicleParameters g_vehicleParam = {
    .mass               = 300,
    .wheelbase          = 1.5,
    .trackWidthFront    = 0.8,
    .trackWidthRear     = 0.8,
    .cgHeight           = 0.28,
    .frontalArea        = 2.2,
    .dragCoefficient    = 0.3,
    .momentOfInertia    = 300*1.5*1.5,
    .wheelRadius        = 0.3,
    .powertraingearRatio= 3.5,
    .weightBiasFront    = 0.55,
    .steeringRatio      = 12
};