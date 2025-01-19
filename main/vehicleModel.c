#include <stdio.h>
#include "lateralDynamics.h"

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

}