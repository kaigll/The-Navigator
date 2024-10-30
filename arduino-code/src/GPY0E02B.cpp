/*
    Custom library for GPY0E02B IR sensor
*/

#include "GPY0E02B.h"

// Initialize I2C
mbed::I2C i2c(P0_31, P0_2);

GPY0E02B::GPY0E02B() {}

void GPY0E02B::selectBus(int bus) {
  const char mux_cmd = 1 << bus;
  const char mux_addr = 0xEE;
  i2c.write(mux_addr, &mux_cmd, 1);
}

float GPY0E02B::measureDistanceCm() {
  char cmd = 0x5E; // Register address from the task
  char data[2];
  byte sensorAddress = 0x80;

  i2c.write(sensorAddress, &cmd, 1);
  wait_us(500); // Provided delay
  i2c.read(sensorAddress, data, 2);

  int distance = (data[0] << 8) | data[1];
  return distance *0.001; // confusing scale factor of 1000?????
}
