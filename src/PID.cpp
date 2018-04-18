//
// Created by Oleksandra Baukh on 4/10/18.
//

#include "PID.h"

float mark_os::algorithms::PID::calculate(float currentFunctionValue, uint16 t) {
    auto error = targetValue - currentFunctionValue;
    auto proportional = error * kp;

    integralOfError += ki * error;

    float derivative = 0.0;
    if (previousError != 0) {
        derivative = kd * (error - previousError) / (t - previousT);
    }
    previousError = error;
    previousT = t;

    return proportional + integralOfError + derivative;
}

mark_os::algorithms::PID::PID(float kp, float ki, float kd, float targetValue)
        : kp(kp), ki(ki), kd(kd), targetValue(targetValue) {}
