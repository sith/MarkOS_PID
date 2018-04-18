//
// Created by Oleksandra Baukh on 4/10/18.
//

#ifndef MARKOS_MAIN_PID_H
#define MARKOS_MAIN_PID_H


#include <types.h>

namespace mark_os {
    namespace algorithms {
        class PID {
            float kp;
            float ki;
            float kd;
            float targetValue;
            float integralOfError = 0;
            float previousError = 0;
            uint16 previousT = 0;
        public:

            PID(float kp, float ki, float kd, float targetValue);

            float calculate(float currentFunctionValue, uint16 t);
        };


    }
}

#endif //MARKOS_MAIN_PID_H
