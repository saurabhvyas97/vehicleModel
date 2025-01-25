// Header file for Longitudinal Dynamics: longitudinalDynamics.h

#ifndef LONGITUDINAL_DYNAMICS_H
#define LONGITUDINAL_DYNAMICS_H

//Define structure for longitudinal dynamics outputs
typedef struct
{
    float   longitudinalAcceleration;   //Longitudinal acceleration of the vehicle in m/s^2
    float   longitudinalVelocity;       //Longitudinal velocity of the vehicle in m/s
    float   slipRatioFrontInner;        //Slip ratio of the front inner tires
    float   slipRatioFrontOuter;        //Slip ratio of the front outer tires
    float   slipRatioRearInner;         //Slip ratio of the rear inner tires
    float   slipRatioRearOuter;         //Slip ratio of the rear outer tires
} LongitudinalDynamics;


#endif