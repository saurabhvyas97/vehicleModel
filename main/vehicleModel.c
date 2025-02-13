#include <stdio.h>
#include "tireModel.h"
#include "vehicleConfig.h"
#include "simulationConfig.h"
#include "lateralDynamics.h"
#include "longitudinalDynamics.h"
#include "drivingCommands.h"

void main()
{
    TireParameters  tireParam;
    TireInputs      tireInput;
    TireOutputs     tireOutput;
    LateralDynamics latDyn;

    //Initialize tire parameters
    tireParam.mu                = 3.09060945;
    tireParam.stiffnessFactor   = 0.7444954026;
    tireParam.shapeFactor       = 0.2442768454;
    tireParam.peakForce         = 1.364104796;

    LongitudinalDynamics longDyn;
    DrivingCommands drivingCmd;

    drivingCmd.steeringAngle = 20/57.3;
    longDyn.longitudinalVelocity = 0.5/3.6;
    latDyn.lateralAcceleration = 0.0;

    float steeringIncrement = 0.0;
    float longvelIncrement = 0.01/3.6;
    float simTime = 0;
    while (simTime<g_simulationParam.endTime)
    {   
        calculateWheelLoads(&latDyn);
        calculateSlipAngles(&longDyn, &latDyn, &drivingCmd);    
        calculateVehicleTireForces(&tireParam, &latDyn, &tireInput, &tireOutput);

        //Calculate lateral dynamics
        calculateLateralDynamics(&tireOutput, &latDyn, &longDyn);
 
        //Update simulation time
        simTime += g_simulationParam.timeStep;

        //Increase the steering angle if steering is below 20 degrees
        if (drivingCmd.steeringAngle < g_vehicleParam.maxSteeringAngle*g_simulationParam.deg2rad)
        {
            drivingCmd.steeringAngle += (g_vehicleParam.maxSteeringAngle-drivingCmd.steeringAngle)/g_vehicleParam.maxSteeringAngle*steeringIncrement*g_simulationParam.deg2rad;
        }
        else
        {
            drivingCmd.steeringAngle = drivingCmd.steeringAngle;
        }

        //Increase the longitudinal velocity if velocity is below 40 m/s
        if (longDyn.longitudinalVelocity < g_vehicleParam.maxLongitudinalVelocity)
        {
            longDyn.longitudinalVelocity += (g_vehicleParam.maxLongitudinalVelocity-longDyn.longitudinalVelocity)/g_vehicleParam.maxLongitudinalVelocity*longvelIncrement;
        }
        else
        {
            longDyn.longitudinalVelocity = longDyn.longitudinalVelocity;
        }
        
        // Write the output to the file
        fprintf(outputFile, "%f,%f,%f,%f,%f,%f,%f,%f\n", simTime, drivingCmd.steeringAngle, latDyn.lateralAcceleration,latDyn.slipAngleFrontLeft,tireOutput.lateralForceFrontRight,latDyn.normalForceFrontRight,latDyn.bodySlipAngle,longDyn.longitudinalVelocity);
    }
    fclose(outputFile);

    printf("Simulation complete. Results written to tire_forces.csv\n");

}