#ifndef DRIVING_COMMANDS_H
#define DRIVING_COMMANDS_H


typedef enum
{
    DRIVE,
    REVERSE,
    NEUTRAL
} DrivingDirection_E;

//Define structure for driving commands
typedef struct
{
    float               throttle;       //Throttle input in percentage
    float               brake;          //Brake input in percentage
    float               steeringAngle;  //Steering angle in rad
    DrivingDirection_E  direction; //Driving direction
} DrivingCommands;

#endif