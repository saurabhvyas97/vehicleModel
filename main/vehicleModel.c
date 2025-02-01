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
    FILE *outputFile;


    // Open the file to write the results
    outputFile = fopen("tire_forces.csv", "w");
    if (outputFile == NULL) {
        printf("Error opening file!\n");
        return;
    }

    // Write the header to the file
    fprintf(outputFile, "SteeringAngle,LateralAcc,slip,force,load,bodyslip,longVelocity\n");

    //Initialize tire parameters
    tireParam.mu                = 3.09060945;
    tireParam.stiffnessFactor   = 0.7444954026;
    tireParam.shapeFactor       = 0.2442768454;
    tireParam.peakForce         = 1.364104796;

    /*
    //Initialize tire inputs
    tireInput.slipAngle         = -5;
    tireInput.normalForce       = -650;

    // Simulate slip angles from -12 to 12 with 0.1 steps
    for (float slipAngle = -12.0; slipAngle <= 12.0; slipAngle += 0.1) {
        // Initialize tire inputs
        tireInput.slipAngle     = slipAngle;
        tireInput.normalForce   = -650;

        // Calculate tire forces
        calculateTireForces(&tireParam, &tireInput, &tireOutput);

        
    }
    */

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
        /*
        printf("Wheel loads calculated\n");
        printf("Normal force front Left: %f\n", latDyn.normalForceFrontLeft);
        printf("Normal force front Right: %f\n", latDyn.normalForceFrontRight);
        printf("Normal force rear Left: %f\n", latDyn.normalForceRearLeft);
        printf("Normal force rear Right: %f\n", latDyn.normalForceRearRight);
        */

        calculateSlipAngles(&longDyn, &latDyn, &drivingCmd);
        /*
        printf("Slip angles calculated\n");
        printf("Slip angle front Left: %f\n", latDyn.slipAngleFrontLeft);
        printf("Slip angle front Right: %f\n", latDyn.slipAngleFrontRight);
        printf("Slip angle rear Left: %f\n", latDyn.slipAngleRearLeft);
        printf("Slip angle rear Right: %f\n", latDyn.slipAngleRearRight);
        */
    
        calculateVehicleTireForces(&tireParam, &latDyn, &tireInput, &tireOutput);
        /*
        printf("TireForceFrontLeft %f\n", tireOutput.lateralForceFrontLeft);
        printf("TireForceFrontRight %f\n", tireOutput.lateralForceFrontRight);
        printf("TireForceRearLeft %f\n", tireOutput.lateralForceRearLeft);
        printf("TireForceRearRight %f\n", tireOutput.lateralForceRearRight);
        */
        

        //Calculate lateral dynamics
        calculateLateralDynamics(&tireOutput, &latDyn, &longDyn);
        //printf("Lateral acceleration is: %f\n", latDyn.lateralAcceleration);

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
        fprintf(outputFile, "%f,%f,%f,%f,%f,%f,%f\n", drivingCmd.steeringAngle, latDyn.lateralAcceleration,latDyn.slipAngleFrontLeft,tireOutput.lateralForceFrontRight,latDyn.normalForceFrontRight,latDyn.bodySlipAngle,longDyn.longitudinalVelocity);
    }
    fclose(outputFile);

    printf("Simulation complete. Results written to tire_forces.csv\n");

}