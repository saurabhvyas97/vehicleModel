//C File: vehicleConfig.c
#include "vehicleConfig.h"

//Initalize vehicle parameters
VehicleParameters g_vehicleParam = {
    .mass               = 1500,
    .wheelbase          = 2.5,
    .trackWidthFront    = 1.5,
    .trackWidthRear     = 1.5,
    .cgHeight           = 0.5,
    .frontalArea        = 2.2,
    .dragCoefficient    = 0.3,
    .momentOfInertia    = 1500,
    .wheelRadius        = 0.3,
    .powertraingearRatio= 3.5,
    .weightBiasFront    = 0.55,
    .steeringRatio      = 12
};