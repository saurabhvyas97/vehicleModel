#include <stdio.h>
#include "tireModel.h"
#include "vehicleConfig.h"
#include "simulaltionConfig.h"
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
    fprintf(outputFile, "SlipAngle,LateralForce\n");

    //Initialize tire parameters
    tireParam.mu                = 3.09060945;
    tireParam.stiffnessFactor   = 0.7444954026;
    tireParam.shapeFactor       = 0.2442768454;
    tireParam.peakForce         = 1.364104796;

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

        // Write the output to the file
        fprintf(outputFile, "%f,%f\n", slipAngle, tireOutput.lateralForce);
    }

    fclose(outputFile);

    printf("Simulation complete. Results written to tire_forces.csv\n");


    printf("Vehicle mass: %f\n", g_vehicleParam.mass);
    calculateWheelLoads(&latDyn);
    printf("Lateral acceleration: %f\n", latDyn.lateralAcceleration);
    printf("Normal force front inner: %f\n", latDyn.normalForceFrontInner);

    LongitudinalDynamics longDyn;
    DrivingCommands drivingCmd;
    VehicleTireOutputs vehicleTireOutput;

    drivingCmd.steeringAngle = 0/57.3;
    longDyn.longitudinalVelocity = 40/3.6;

    float simTime = 0;
    while (simTime<g_simulationParam.endTime)
    {   
        calculateWheelLoads(&latDyn);
        calculateSlipAngles(&longDyn, &latDyn, &drivingCmd);
    
        calculateVehicleTireForces(&tireParam, &latDyn, &tireInput, &vehicleTireOutput, &tireOutput);

        //Calculate lateral dynamics
        calculateLateralDynamics(&tireOutput);

        //Update simulation time
        simTime += g_simulationParam.timeStep;
    }

}