/*
    Custom library for TB6612FNG Motor
*/

#ifndef MOTOR_H
#define MOTOR_H

#include <mbed.h>

class Motor {
public:
    /**
     * @brief Construct a new Motor object
     *
     * @param motorADir Pin for motor A direction control
     * @param motorBDir Pin for motor B direction control
     * @param motorAPWM Pin for motor A PWM speed control
     * @param motorBPWM Pin for motor B PWM speed control
     */
    Motor(PinName motorADir, PinName motorBDir, PinName motorAPWM,
          PinName motorBPWM);

    /**
     * @brief Update the direction and speed of both motors simultaneously
     *
     * @param directionA Direction for motor A (e.g., 1 or 0)
     * @param speedA Speed for motor A (0.0f to 1.0f)
     * @param directionB Direction for motor B (e.g., 1 or 0)
     * @param speedB Speed for motor B (0.0f to 1.0f)
     */
    void updateMotors(int directionA, int directionB, float speedA,
                      float speedB);

    /**
     * @brief Stop motor A by setting its speed to 0
     */
    void stopMotorA();

    /**
     * @brief Stop motor B by setting its speed to 0
     */
    void stopMotorB();

private:
    mbed::DigitalOut motorADir; // Digital pin for motor A direction control
    mbed::PwmOut motorAPWM;     // PWM pin for motor A speed control
    mbed::DigitalOut motorBDir; // Digital pin for motor B direction control
    mbed::PwmOut motorBPWM;     // PWM pin for motor B speed control
    rtos::Mutex mutex;          // Single mutex to ensure atomic updates
};

#endif
