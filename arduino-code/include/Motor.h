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
     * @brief Setup PWM for motor A and motor B
     */
    void setup();

    /**
     * @brief Update the direction and speed of motor A
     *
     * @param direction Direction for motor (e.g., 1 or 0)
     * @param speed Speed for motor (0.0f to 1.0f)
     */
    void updateMotorA(int direction, float speed);

    /**
     * @brief Update the direction and speed of motor B
     *
     * @param direction Direction for motor (e.g., 1 or 0)
     * @param speed Speed for motor (0.0f to 1.0f)
     */
    void updateMotorB(int direction, float speed);

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
     * @brief Update the direction and speed of both motors simultaneously
     *
     * @param speedA Speed for motor A (0.0f to 1.0f)
     * @param speedB Speed for motor B (0.0f to 1.0f)
     */
    void updateMotors(float speedA, float speedB);

    /**
     * @brief Synchronise the motors by pausing movement if one motor is detected to be going faster than the other
     */
    void syncMotors();

    /**
     * @brief Stop motor A by setting its speed to 0
     */
    void stopMotorA();

    /**
     * @brief Stop motor B by setting its speed to 0
     */
    void stopMotorB();

    /**
     * @brief Stop motor B by setting its speed to 0
     */
    void stopMotors();

    /**
     * @brief Attach the interrupt for both motor encoders
     */
    void startCounting();

    /**
     * @brief Reset both encoder counts to zero
     */
    void resetCount();

    /**
     * @brief Increment encoderCountA by 1, plus or minus depending on motor direction
     */
    void countPulseA();

    /**
     * @brief Increment encoderCountB by 1, plus or minus depending on motor direction
     */
    void countPulseB();

    /**
     * @brief Converts encoder count of motor A to distance in cm
     * @returns Distance in cm
     */
    float calculateDistanceA();

    /**
     * @brief Converts encoder count of motor B to distance in cm
     * @returns Distance in cm
     */
    float calculateDistanceB();

    float calculateSpeedA();

    float calculateSpeedB();

private:
    mbed::DigitalOut motorADir; // Digital pin for motor A direction control
    mbed::DigitalOut motorBDir; // Digital pin for motor B direction control
    mbed::PwmOut motorAPWM;     // PWM pin for motor A speed control
    mbed::PwmOut motorBPWM;     // PWM pin for motor B speed control
    rtos::Mutex mutex;          // Mutex to prevent simultaneous calls
    float motorASpeed;          // Stores the current speed of motor A
    float motorBSpeed;          // Stores the current speed of motor B

    long int shaftRevA;
    long int shaftRevB;
    long int encoderCountA; // The encoder count for motor A
    long int encoderCountB; // The encoder count for motor B
    volatile int lastEncoderCountA = 0;
    volatile int lastEncoderCountB = 0;

    const float wheelDiameter = 4.7;                     // Wheel diameter in cm
    const float wheelCircumference = wheelDiameter * PI; // 4.7 * pi =~ 14.7654854719;
    const int pulsesPerRevolution = 330;                 // The number of pulses from the encoder per full wheel revolution

    mbed::Timer timer;
};

#endif
