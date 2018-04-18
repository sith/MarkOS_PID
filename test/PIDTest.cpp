//
// Created by Oleksandra Baukh on 4/10/18.
//

#include <PID.h>
#include "PIDTest.h"

void PIDTest::SetUp() {
    Test::SetUp();
}

TEST_F(PIDTest, P_Test) {
    float kp = 1.2;
    float targetValue = 1.0;
    mark_os::algorithms::PID pController{kp, 0.0, 0.0, targetValue};
    auto currentFunctionValue = 5.0;
    auto u = pController.calculate(currentFunctionValue, 0);
    ASSERT_FLOAT_EQ(u, kp * (targetValue - currentFunctionValue));
}


TEST_F(PIDTest, PI_Test) {
    float kp = 1.2;
    float ki = 1.5;
    float targetValue = 1.0;
    mark_os::algorithms::PID pController{kp, ki, 0.0, targetValue};

    float valueAtT0 = 0.1;
    float valueAtT1 = 0.2;
    float valueAtT2 = 0.3;
    float u0 = pController.calculate(valueAtT0, 0);
    float u1 = pController.calculate(valueAtT1, 1);
    float u2 = pController.calculate(valueAtT2, 2);
    ASSERT_FLOAT_EQ(u0, kp * (targetValue - valueAtT0) + ki * (targetValue - valueAtT0));
    ASSERT_FLOAT_EQ(u1, kp * (targetValue - valueAtT1) + ki * ((targetValue - valueAtT0) + (targetValue - valueAtT1)));
    ASSERT_FLOAT_EQ(u2, kp * (targetValue - valueAtT2) +
                        ki * ((targetValue - valueAtT0) + (targetValue - valueAtT1) + (targetValue - valueAtT2)));

}


TEST_F(PIDTest, PID_Test) {
    float kp = 1.2;
    float ki = 1.5;
    float kd = 1.7;
    float targetValue = 1.0;
    mark_os::algorithms::PID pController{kp, ki, kd, targetValue};

    float valueAtT0 = 0.1;
    float valueAtT1 = 0.2;
    float valueAtT2 = 0.3;
    uint16 t0 = 0;
    uint16 t1 = 2;
    uint16 t2 = 5;
    float u0 = pController.calculate(valueAtT0, t0);
    float u1 = pController.calculate(valueAtT1, t1);
    float u2 = pController.calculate(valueAtT2, t2);
    float error0 = targetValue - valueAtT0;
    float error1 = targetValue - valueAtT1;
    float error2 = targetValue - valueAtT2;
    ASSERT_FLOAT_EQ(u0, kp * error0 + ki * error0);
    ASSERT_FLOAT_EQ(u1, kp * error1 +
                        ki * (error0 + error1) +
                        kd * (error1 - error0) / (t1 - t0));
    ASSERT_FLOAT_EQ(u2, kp * error2 +
                        ki * (error0 + error1 + error2) +
                        kd * (error2 - error1) / (t2 - t1));

}

