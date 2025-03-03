#include <stdio.h>
#include "tireModel.h"
#include "lateralDynamics.h"
#include "simulationTypes.h"

void main()
{
    TireParameters      tireParam;
    TireInputs          tireInput;
    TireOutputs         tireOutput;
    LateralDynamics     latDyn;
    FILE                *outputFile;
    LongitudinalDynamics longDyn;
    DrivingCommands     drivingCmd;


    // Open the file to write the results
    outputFile = fopen("vehicleModelOutput.csv", "w");
    if (outputFile == NULL)
    {
        printf("Error opening file!\n");
        return;
    }

    // Write the header to the file
    fprintf(outputFile, "SimulationTime,SteeringAngle,LateralAcc,slip,force,load,bodyslip,longVelocity,yawRate\n");

    //Initialize tire parameters
    tireParam.mu                = 3.09060945;
    tireParam.stiffnessFactor   = 0.7444954026;
    tireParam.shapeFactor       = 0.2442768454;
    tireParam.peakForce         = 1.364104796;

    //Initialize simulation parameters
    drivingCmd.steeringAngle = 0/57.3;
    longDyn.longitudinalVelocity = 50/3.6;
    latDyn.lateralAcceleration = 0.0;
    float steeringIncrement = 0.005;
    float longvelIncrement = 0.1/3.6;
    float simTime = 0;
    float nextLoggingTime = g_simulationParam.logInterval;

/********************************************************************************************************************/
    //Vehicle simulation loop
    while (simTime<g_simulationParam.endTime)
    {   
        //Calculate normal forces for new simulation step
        calculateWheelLoads_LatLT(&latDyn);
     
        //Calculate slip angles for new simulation step
        calculateSlipAngles(&longDyn, &latDyn, &drivingCmd);
  
        //Calculate tire forces for new simulation step
        calculateVehicleTireForces(&tireParam, &latDyn, &tireInput, &tireOutput);
    
        //Calculate lateral dynamics - lateral acceleration, yaw rate, body slip angle
        calculateLateralDynamics(&tireOutput, &latDyn, &longDyn);
        //printf("Lateral acceleration is: %f\n", latDyn.lateralAcceleration);

        //Update simulation time
        simTime += g_simulationParam.timeStep;

        //Increase the steering angle if steering is below 20 degrees
        if (drivingCmd.steeringAngle < g_vehicleParam.maxSteeringAngle*g_simulationParam.deg2rad)
        {
            drivingCmd.steeringAngle += (g_vehicleParam.maxSteeringAngle-drivingCmd.steeringAngle)/
                                         g_vehicleParam.maxSteeringAngle*steeringIncrement*g_simulationParam.deg2rad;
        }
        else
        {
            drivingCmd.steeringAngle = drivingCmd.steeringAngle;
        }

        //Increase the longitudinal velocity if velocity is below 40 m/s
        if (longDyn.longitudinalVelocity < g_vehicleParam.maxLongitudinalVelocity)
        {
            longDyn.longitudinalVelocity += (g_vehicleParam.maxLongitudinalVelocity-longDyn.longitudinalVelocity)/
                                             g_vehicleParam.maxLongitudinalVelocity*longvelIncrement;
        }
        else
        {
            longDyn.longitudinalVelocity = longDyn.longitudinalVelocity;
        }

        // Log the data at the specified logging interval
        if (simTime >= nextLoggingTime)
        {
            fprintf(outputFile, "%f,%f,%f,%f,%f,%f,%f,%f,%f\n", simTime, drivingCmd.steeringAngle, 
            latDyn.lateralAcceleration,latDyn.slipAngleFrontLeft,tireOutput.lateralForceFrontRight,
            latDyn.normalForceFrontRight,latDyn.bodySlipAngle,longDyn.longitudinalVelocity, latDyn.yawRate);

            // Update the next logging time
            nextLoggingTime += g_simulationParam.logInterval;
        }
        
    }
/********************************************************************************************************************/
    // Close the file ans print results.
    fclose(outputFile);
    printf("Simulation complete. Results written to vehicleModelOutput.csv\n");

}