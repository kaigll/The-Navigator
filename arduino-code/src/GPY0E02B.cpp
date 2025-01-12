/*
    Custom library for GPY0E02B IR sensor
*/

#include "GPY0E02B.h"

// Initialize I2C
mbed::I2C i2c(P0_31, P0_2);

IRSensor::IRSensor() {}

void IRSensor::selectBus(int bus) {
    mutex.lock();
    const char mux_cmd = 1 << bus;
    const char mux_addr = 0xEE;
    i2c.write(mux_addr, &mux_cmd, 1);
    mutex.unlock();
}

float IRSensor::measureDistanceCm() {
    mutex.lock();
    char cmd = 0x5E; // Register address from the task
    char data[2];
    byte sensorAddress = 0x80;

    i2c.write(sensorAddress, &cmd, 1);
    wait_us(500); // Provided delay
    i2c.read(sensorAddress, data, 2);

    uint16_t combinedData = (data[0] << 4) | data[1];
    float distance = static_cast<float>(combinedData) / 64.0;

    mutex.unlock();
    return distance;
}

float IRSensor::measureDistanceCm(int bus) {
    mutex.lock();
    const char mux_cmd = 1 << bus; // select the specified i2c bus
    const char mux_addr = 0xEE;
    i2c.write(mux_addr, &mux_cmd, 1);

    char cmd = 0x5E; // Register address from the task
    char data[2];
    byte sensorAddress = 0x80;

    i2c.write(sensorAddress, &cmd, 1);
    wait_us(500); // Provided delay
    i2c.read(sensorAddress, data, 2);

    uint16_t combinedData = (data[0] << 4) | data[1];
    float distance = static_cast<float>(combinedData) / 64.0;

    mutex.unlock();
    return distance;
}