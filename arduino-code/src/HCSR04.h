/*
  HCSR04 - Library for arduino, for HC-SR04 ultrasonic distance sensor.
  Created by Martin Sosic, June 11, 2016.

  Source: https://github.com/Martinsos/arduino-lib-hc-sr04

  Modified by Kai Gledhill-Lawson to accmomodate for a bridged trigger and echo pin
*/

#ifndef HCSR04_H
#define HCSR04_H

#include <Arduino.h>

class UltraSonicDistanceSensor {
 public:
    /**
     * @param triggerPin  Digital pin that is used for controlling sensor (output).
     * @param maxDistanceCm  Maximum distance sensor can measure, defaults to 4m for HC-SR04.
     *                       You might want to set this value if you are using different sensor than HC-SR04
     *                       or if you don't care about distances larger than whatever you will set it to
     *                       (therefore reducing time it takes for a single measurement).
     * @param maxTimeoutMicroSec  Single measurement will never take longer than whatever value you provide here.
     *   You might want to do this in order to ensure your program is never blocked for longer than some specific time,
     *   since measurements are blocking.
     *   By default, there is no limit on time (only on distance). By defining timeout, you are again limiting the distance. 
     */
    UltraSonicDistanceSensor(byte triggerPin, unsigned short maxDistanceCm = 400, unsigned long maxTimeoutMicroSec = 0);

    /**
     * Measures distance by sending ultrasonic waves and measuring time it takes them
     * to return.
     * @returns Distance in centimeters, or negative value if distance is greater than 400cm.
     */
    float measureDistanceCm();

    /**
     * Measures distance by sending ultrasonic waves and measuring time it takes them
     * to return. Measurement can not exceed duration calculated with maxDistanceCm or maxTimeoutMicroSec.
     * @param temperature  Temperature in degrees celsius
     * @returns Distance in centimeters, or negative value if distance is greater than 400cm.
     */
    float measureDistanceCm(float temperature);
 private:
    byte triggerPin;
    unsigned short maxDistanceCm;
    unsigned long maxTimeoutMicroSec;
};

#endif // HCSR04_H