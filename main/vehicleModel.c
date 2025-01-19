#include <stdio.h>
#include "tireModel.h"

void main()
{
    TireParameters  tireParam;
    TireInputs      tireInput;
    TireOutputs     tireOutput;

    //Initialize tire parameters
    tireParam.mu            = 0.8;
    tireParam.stiffnessFactor = 100000;
    tireParam.shapeFactor   = 1.0;

    //Initialize tire inputs
    tireInput.slipAngle     = 0.1;
    tireInput.normalForce   = 1000;

    //Calculate tire forces
    calculateTireForces(&tireParam, &tireInput, &tireOutput);

    //Print the output
    printf("Lateral Force: %f\n", tireOutput.lateralForce);
    printf("Longitudinal Force: %f\n", tireOutput.longForce);
}