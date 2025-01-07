#ifndef MAP_H
#define MAP_H

#include <Arduino.h>
#include <cmath>
#include <cstdint>

// Constants representing different points on the map
const uint8_t FREE_SPACE = 0;
const uint8_t OBSTACLE = 1;
const uint8_t SEEN_LOCATION = 2;
const uint8_t ROBOT_LOCATION = 3;

class Map {
public:
    Map(int width, int height, int cellSize);
    ~Map();
    void setCell(int x, int y, uint8_t value);
    uint8_t getCell(int x, int y);
    void printGrid(); // Debugging method

    // Robot-related methods
    void setRobotPosition(int x, int y, int angle);
    void identifyStartPosition(float dB, float dBR);

    void moveRobotForward(int distance);
    void rotateRobotLeft(int degrees);
    void rotateRobotRight(int degrees);
    int getRobotX();
    int getRobotY();
    int getRobotAngle();

    std::pair<int, int> calculateGlobalPosition(float offsetX, float offsetY, float distance, float sensorAngle);
    void updateGridWithSensor(float sensorX, float sensorY, float distance, float sensorAngle);
    void updateGrid(float dLF, float dLB, float dRF, float dRB, float dF, float dB);
    void updateGrid(float dLF, float dLB, float dRF, float dRB);

private:
    uint8_t *grid;
    int width;
    int height;
    int cellSize;
    int robotX;
    int robotY;
    int robotAngle; // Angle in degrees (0 - 359)
    void initGrid();
    void markPath(int x0, int y0, int x1, int y1);

    /*
     * Sensor locations relative to the center of the robot
     */
    const float sensorLF_X = -7.75; // Left front sensor X offset
    const float sensorLF_Y = 10.0;  // Left front sensor Y offset
    const float sensorLB_X = -7.75; // Left back sensor X offset
    const float sensorLB_Y = -10.0; // Left back sensor Y offset
    const float sensorRF_X = 7.75;  // Right front sensor X offset
    const float sensorRF_Y = 10.0;  // Right front sensor Y offset
    const float sensorRB_X = 7.75;  // Right back sensor X offset
    const float sensorRB_Y = -10.0; // Right back sensor Y offset
    const float sensorF_X = 0.0;    // Front sensor X offset
    const float sensorF_Y = 10.0;   // Front sensor Y offset
    const float sensorB_X = 0.0;    // Back sensor X offset
    const float sensorB_Y = -10.0;  // Back sensor Y offset
};

#endif // MAP_H
