#include <stdio.h>
#include "tireModel.h"

void main()
{
    TireParameters  tireParam;
    TireInputs      tireInput;
    TireOutputs     tireOutput;

    //Initialize tire parameters
    tireParam.mu                = 1.33;
    tireParam.stiffnessFactor   = 0.05;
    tireParam.shapeFactor       = 0.1335;
    tireParam.peakForce         = 1.0;

    //Initialize tire inputs
    tireInput.slipAngle         = -5;
    tireInput.normalForce       = -650;

    //Calculate tire forces
    calculateTireForces(&tireParam, &tireInput, &tireOutput);

    //Print the output
    printf("Lateral Force: %f\n", tireOutput.lateralForce);
    printf("Longitudinal Force: %f\n", tireOutput.longForce);

}