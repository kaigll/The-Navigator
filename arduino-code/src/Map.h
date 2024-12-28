#ifndef MAP_H
#define MAP_H

#include <Arduino.h>
#include <cmath>
#include <cstdint>

// Constants representing different points on the map
const uint8_t FREE_SPACE = 0;
const uint8_t OBSTACLE = 1;
const uint8_t ROBOT_LOCATION = 2;

class Map {
public:
    Map(int width, int height, int cellSize);
    ~Map();
    void setCell(int x, int y, uint8_t value);
    uint8_t getCell(int x, int y);
    void printGrid(); // Debugging method

    // Robot-related methods
    void setRobotPosition(int x, int y, int angle);
    void moveRobotForward(int distance);
    void rotateRobotLeft(int degrees);
    void rotateRobotRight(int degrees);
    int getRobotX();
    int getRobotY();
    int getRobotAngle();

    void updateGridWithSensorData(float distance, int sensorOffsetX, int sensorOffsetY, int sensorAngle);

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
};

#endif // MAP_H
