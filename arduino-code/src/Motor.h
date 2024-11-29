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

    void setup();

    /**
     * @brief Update the direction and speed of both motors simultaneously
     *
     * @param directionA Direction for motor A (e.g., 1 or 0)
     * @param directionB Direction for motor B (e.g., 1 or 0)
     * @param speedA Speed for motor A (0.0f to 1.0f)
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

    void startCounting();
    
    void resetCount();

    void countPulseA();

    void countPulseB();

    float calculateDistanceA();

    float calculateDistanceB();

    float calculateSpeedA();

    float calculateSpeedB();

private:
    mbed::DigitalOut motorADir; // Digital pin for motor A direction control
    mbed::DigitalOut motorBDir; // Digital pin for motor B direction control
    mbed::PwmOut motorAPWM;     // PWM pin for motor A speed control
    mbed::PwmOut motorBPWM;     // PWM pin for motor B speed control
    rtos::Mutex mutex;          // Single mutex to ensure atomic updates

    
    long int shaftRevA;
    long int shaftRevB;
    long int encoderCountA;
    long int encoderCountB;
    volatile int lastEncoderCountA = 0;
    volatile int lastEncoderCountB = 0;

    const float wheelDiameter = 4.7;
    const float wheelCircumference = wheelDiameter * PI; //14.7654854719; // wheelDiameter * pi
    const int pulsesPerRevolution = 330; 

    mbed::Timer timer;
};

#endif
