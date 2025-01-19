#include <CUnit/CUnit.h>
#include <CUnit/Basic.h>
#include "tireModel.h"

void test_tireModel_checkSign(void) {
    
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

    CU_ASSERT(tireOutput.lateralForce > 0);
    
    //Initialize tire inputs
    tireInput.slipAngle         = 5;
    tireInput.normalForce       = -650;

    //Calculate tire forces
    calculateTireForces(&tireParam, &tireInput, &tireOutput);

    CU_ASSERT(tireOutput.lateralForce < 0);
}

int main() {
    CU_initialize_registry();
    CU_pSuite suite = CU_add_suite("TireModel Suite", NULL, NULL);
    CU_add_test(suite, "Test TireModel Sign Check", test_tireModel_checkSign);
    
    CU_basic_set_mode(CU_BRM_VERBOSE);
    CU_basic_run_tests();
    CU_cleanup_registry();
    return 0;
}