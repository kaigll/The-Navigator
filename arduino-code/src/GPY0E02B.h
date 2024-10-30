/*
    Custom library for GPY0E02B IR sensor
*/

#ifndef GPY0E02B_H
#define GPY0E02B_H

#include <mbed.h>
#include <Arduino.h>

class GPY0E02B {
  public:
    GPY0E02B();

  /**
   * Select the appropriate bus on the I2C multiplexer. 
   * @param bus The bus number to select (0-3). 
   * */
    void selectBus(int bus);

    /** 
     * Measure the distance by triggering the sensor and reading the returned echo.
     * @returns Distance in centimeters, or -1 if an error occurs.
     */
    float measureDistanceCm();

  private:
    byte _address;
};

#endif