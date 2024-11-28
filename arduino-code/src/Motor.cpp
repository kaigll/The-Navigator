/*
    Custom library for TB6612FNG Motor
*/

#include "Motor.h"

mbed::InterruptIn encoderA(P1_11);
mbed::InterruptIn encoderB(P1_12);

Motor::Motor(PinName motorADir, PinName motorBDir, PinName motorAPWM,
             PinName motorBPWM)
    : motorADir(motorADir), motorBDir(motorBDir), motorAPWM(motorAPWM),
      motorBPWM(motorBPWM) {}

void Motor::setup() {
    motorAPWM.period_ms(1);
    motorBPWM.period_ms(1);
    Serial.println("Motor A PWM period set to 1 ms");
    Serial.println("Motor B PWM period set to 1 ms");
}

void Motor::updateMotors(int directionA, int directionB, float speedA,
                         float speedB) {
    mutex.lock();
    motorADir = directionA;
    motorBDir = directionB;
    motorAPWM.write(speedA);
    motorBPWM.write(speedB);
    mutex.unlock();
}

void Motor::stopMotorA() { motorAPWM.write(0.0f); }

void Motor::stopMotorB() { motorBPWM.write(0.0f); }

void Motor::startCounting() {
    timer.start();
    encoderA.rise(mbed::callback(this, &Motor::countPulseA));
    encoderB.rise(mbed::callback(this, &Motor::countPulseB));
}

void Motor::countPulseA() {
    if (motorADir == 0)
        encoderCountA++;
    else
        encoderCountA--;
}

void Motor::countPulseB() {
    if (motorBDir == 0)
        encoderCountB++;
    else
        encoderCountB--;
}

float Motor::calculateDistanceA() {
    return (encoderCountA * wheelCircumference) / pulsesPerRevolution;
}
float Motor::calculateDistanceB() {
    return (encoderCountB * wheelCircumference) / pulsesPerRevolution;
}

float Motor::calculateSpeedA() {
    float deltaTime = timer.read();
    timer.reset();

    int deltaPulsesA = encoderCountA - lastEncoderCountA;

    lastEncoderCountA = encoderCountA;
    float distanceA = (deltaPulsesA * wheelCircumference) / pulsesPerRevolution;
    return distanceA / deltaTime;
}

float Motor::calculateSpeedB() {
    float deltaTime = timer.read();
    timer.reset();

    int deltaPulsesB = encoderCountB - lastEncoderCountB;

    lastEncoderCountB = encoderCountB;
    float distanceB = (deltaPulsesB * wheelCircumference) / pulsesPerRevolution;
    return distanceB / deltaTime;
}