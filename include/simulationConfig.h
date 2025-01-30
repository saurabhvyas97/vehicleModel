#ifndef SIMULATIONCONFIG_H
#define SIMULATIONCONFIG_H

//Define the structure for simulation parameters
typedef struct
{
    float   timeStep;               //Time step for the simulation
    float   endTime;                //End time for the simulation
    float   initialVelocity;        //Initial velocity of the vehicle
    float   initialYawRate;         //Initial yaw rate of the vehicle
    float   initialWheelAngle;      //Initial steering wheel angle
} SimulationParameters;

extern SimulationParameters g_simulationParam;

#endif