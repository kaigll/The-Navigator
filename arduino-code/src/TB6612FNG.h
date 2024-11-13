/*
    Custom library for TB6612FNG Motor Controller
*/

#ifndef TB6612FNG_H
#define TB6612FNG_H

#include <mbed.h>

// MOTOR DIR A = P0_04
// MOTOR DIR B = P0_05
// MOTOR PWR A = P0_27
// MOTOR PWR B = P1_02

class TB6612FNG {
    public:
        TB6612FNG();
};

#endif