#include "logging.h"


void setupLoggingFile()
{
    
    // Open the file to write the results
    outputFile = fopen("vehicleModelLog.csv", "w");
    if (outputFile == NULL) {
        printf("Error opening file!\n");
        return;
    }

    // Write the header to the file
    fprintf(outputFile, "Time,SteeringAngle,LateralAcc,slip,force,load,bodyslip,longVelocity\n");
}

void logVehicleModel(outputfile)
{
    // Write the output to the file
    fprintf(outputFile, "%f,%f,%f,%f,%f,%f,%f,%f\n", simTime, drivingCmd.steeringAngle, latDyn.lateralAcceleration,latDyn.slipAngleFrontLeft,tireOutput.lateralForceFrontRight,latDyn.normalForceFrontRight,latDyn.bodySlipAngle,longDyn.longitudinalVelocity);
}

