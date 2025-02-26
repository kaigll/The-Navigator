/**
 *   Custom library for TB6612FNG Motor
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

void Motor::updateMotorA(int direction, float speed) {
    mutex.lock();
    motorADir = direction;
    motorASpeed = speed;
    motorAPWM.write(speed);
    mutex.unlock();
}

void Motor::updateMotorB(int direction, float speed) {
    mutex.lock();
    motorBDir = direction;
    motorBSpeed = speed;
    motorBPWM.write(speed);
    mutex.unlock();
}

void Motor::updateMotors(int directionA, int directionB, float speedA,
                         float speedB) {
    mutex.lock();
    motorADir = directionA;
    motorBDir = directionB;
    motorASpeed = speedA;
    motorBSpeed = speedB;
    motorAPWM.write(speedA);
    motorBPWM.write(speedB);
    mutex.unlock();
}

void Motor::updateMotors(float speedA, float speedB) {
    mutex.lock();
    motorAPWM.write(speedA);
    motorBPWM.write(speedB);
    mutex.unlock();
}

void Motor::syncMotors() {
    // if motor A is faster than motor B, stop motor A until distance has equalized
    // else if motor B is faster than motor A, stop motor B until distance has equalized
    Util::beginTimeout(timeout, timeoutOccurred, 5.0);
    if (abs(encoderCountA) > abs(encoderCountB)) {
        stopMotorA();
        while (abs(encoderCountA) > abs(encoderCountB)) {
            if (timeoutOccurred) {
                break;
            }
            // do nothing until motor B catches up
            thread_sleep_for(1);
        }
        updateMotorA(motorADir, motorASpeed);
    } else if (abs(encoderCountA) < abs(encoderCountB)) {
        stopMotorB();
        while (abs(encoderCountA) < abs(encoderCountB)) {
            if (timeoutOccurred) {
                break;
            }
            // do nothing until motor A catches up
            thread_sleep_for(1);
        }
        updateMotorB(motorBDir, motorBSpeed);
    }
}

void Motor::stopMotorA() {
    motorAPWM.write(0.0f);
}

void Motor::stopMotorB() {
    motorBPWM.write(0.0f);
}

void Motor::stopMotors() {
    motorAPWM.write(0.0f);
    motorBPWM.write(0.0f);
}

void Motor::startCounting() {
    // timer.start();
    encoderA.rise(mbed::callback(this, &Motor::countPulseA));
    encoderB.rise(mbed::callback(this, &Motor::countPulseB));
}

void Motor::resetCount() {
    encoderCountA = 0;
    encoderCountB = 0;
}

void Motor::countPulseA() {
    if (motorADir == 0) {
        encoderCountA++;
    } else {
        encoderCountA--;
    }
}

void Motor::countPulseB() {
    if (motorBDir == 0) {
        encoderCountB++;
    } else {
        encoderCountB--;
    }
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