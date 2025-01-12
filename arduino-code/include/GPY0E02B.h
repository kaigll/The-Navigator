/*
    Custom library for GPY0E02B IR sensor
*/

#ifndef GPY0E02B_H
#define GPY0E02B_H

#include <Arduino.h>
#include <mbed.h>
#include <rtos.h>

class IRSensor {
public:
    IRSensor();

    /**
     * Select the appropriate bus on the I2C multiplexer.
     * @param bus The bus number to select (0-3).
     * */
    void selectBus(int bus);

    /**
     * Measure the distance by triggering the sensor and reading the returned
     * echo.
     * @returns Distance in centimeters, or -1 if an error occurs.
     */
    float measureDistanceCm();

    /**
     * Measure the distance by triggering the sensor and reading the returned
     * echo.
     * @param bus The bus number to select (0-3) on the I2C multiplexer.
     * @returns Distance in centimeters, or -1 if an error occurs.
     */
    float measureDistanceCm(int bus);

private:
    rtos::Mutex mutex; // Mutex to avoid simultaneous calls
};

#endif