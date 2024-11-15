#ifndef MOTOR_H
#define MOTOR_H

#include <mbed.h>
#include <Arduino.h>

class Motor
{
public:
    Motor(PinName motorADir, PinName motorBDir, PinName motorAPWM, PinName motorBPWM) 
        : motorADir(motorADir), motorBDir(motorBDir), motorAPWM(motorAPWM), motorBPWM(motorBPWM) {}

    void moveMotorA();
    void moveMotorB();

private:
    // a -> left side
    mbed::DigitalOut motorADir;
    mbed::PwmOut motorAPWM;
    // b -> right side
    mbed::DigitalOut motorBDir;
    mbed::PwmOut motorBPWM;
};

#endif