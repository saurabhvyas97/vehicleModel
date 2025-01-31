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
    fprintf(outputFile, "SteeringAngle,LateralAcc,slip,force,load\n");

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

    drivingCmd.steeringAngle = 0/57.3;
    longDyn.longitudinalVelocity = 40/3.6;
    latDyn.lateralAcceleration = 0.0;

    float simTime = 0;
    while (simTime<g_simulationParam.endTime)
    {   
        calculateWheelLoads(&latDyn);
        printf("Wheel loads calculated\n");
        printf("Normal force front inner: %f\n", latDyn.normalForceFrontInner);
        printf("Normal force front outer: %f\n", latDyn.normalForceFrontOuter);
        printf("Normal force rear inner: %f\n", latDyn.normalForceRearInner);
        printf("Normal force rear outer: %f\n", latDyn.normalForceRearOuter);

        calculateSlipAngles(&longDyn, &latDyn, &drivingCmd);
        /*
        printf("Slip angles calculated\n");
        printf("Slip angle front inner: %f\n", latDyn.slipAngleFrontInner);
        printf("Slip angle front outer: %f\n", latDyn.slipAngleFrontOuter);
        printf("Slip angle rear inner: %f\n", latDyn.slipAngleRearInner);
        printf("Slip angle rear outer: %f\n", latDyn.slipAngleRearOuter);
        */
    
        calculateVehicleTireForces(&tireParam, &latDyn, &tireInput, &tireOutput);
        /*
        printf("TireForceFrontInner %f\n", tireOutput.lateralForceFrontInner);
        printf("TireForceFrontOuter %f\n", tireOutput.lateralForceFrontOuter);
        printf("TireForceRearInner %f\n", tireOutput.lateralForceRearInner);
        printf("TireForceRearOuter %f\n", tireOutput.lateralForceRearOuter);
        */
        

        //Calculate lateral dynamics
        calculateLateralDynamics(&tireOutput, &latDyn, &longDyn);
        //printf("Lateral acceleration is: %f\n", latDyn.lateralAcceleration);

        //Update simulation time
        simTime += g_simulationParam.timeStep;

        //Increase the steering angle if steering is below 20 degrees
        if (drivingCmd.steeringAngle < 15/57.3)
        {
            drivingCmd.steeringAngle += 0.1/57.3;
        }
        else
        {
            drivingCmd.steeringAngle = drivingCmd.steeringAngle;
        }
        
        // Write the output to the file
        fprintf(outputFile, "%f,%f,%f,%f,%f\n", drivingCmd.steeringAngle, latDyn.lateralAcceleration,latDyn.slipAngleFrontInner,tireOutput.lateralForceFrontInner,latDyn.normalForceFrontInner);
    }
    fclose(outputFile);

    printf("Simulation complete. Results written to tire_forces.csv\n");

}