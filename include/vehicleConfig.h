//Header file: tireModel.h
#ifndef VEHICLE_CONFIG_H
#define VEHICLE_CONFIG_H

//Structure for vehicle parameters
typedef struct 
{
    float   mass;                   //Mass of the vehicle in kg
    float   wheelbase;              //Wheelbase of the vehicle in m
    float   trackWidthFront;        //Front track width of the vehicle in m
    float   trackWidthRear;         //Rear track width of the vehicle in m
    float   cgHeight;               //Height of the center of gravity of the vehicle in m
    float   frontalArea;            //Frontal area of the vehicle in m^2
    float   dragCoefficient;        //Drag coefficient of the vehicle
    float   momentOfInertia;        //Moment of inertia of the vehicle in kg-m^2
    float   wheelRadius;            //Radius of the wheel in m
    float   powertraingearRatio;    //Gear ratio of the vehicle's driveline
    float   weightBiasFront;        //Weight bias towards the front of the vehicle
    float   steeringRatio;          //Steering ratio of the vehicle
} VehicleParameters;

//Define external variables
extern VehicleParameters g_vehicleParam;

#endif