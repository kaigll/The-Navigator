/*
    Custom library for TB6612FNG Motor
*/

#include "Motor.h"

mbed::InterruptIn EncA(P1_11);
mbed::InterruptIn EncB(P1_12);

Motor::Motor(PinName motorADir, PinName motorBDir, PinName motorAPWM,
             PinName motorBPWM)
    : motorADir(motorADir), motorBDir(motorBDir), motorAPWM(motorAPWM),
      motorBPWM(motorBPWM) {
    this->motorAPWM.period_ms(1);
    this->motorBPWM.period_ms(1);
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

void Motor::countPulses() {
  EncA.rise(mbed::callback(this, &Motor::countPulseA));
  Serial.println(encoderCountA);
}

void Motor::countPulseA(){
   if(motorADir == 0)
     encoderCountA++;
   else
     encoderCountA--;
}

void Motor::countPulseB(){
   if(motorBDir == 0)
     encoderCountB++;
   else
     encoderCountB--;
}
