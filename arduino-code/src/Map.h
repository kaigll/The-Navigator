#ifndef MAP_H
#define MAP_H

#include <Coordinate.h>
#include <utility>
#include <cstdint>

class Map {
public:
    /**
     * @brief Initialize the grid as all free space, and marks the start location of the robot.
     *
     * @param width The width of the grid along the x axis, measured in amount of cells. (e.g. 14 cells of cellSize = 1cm)
     * @param height The height of the grid along the y axis, measured in amount of cells. (e.g. 14 cells of cellSize = 1cm)
     * @param cellSize The size of a grid cell in cm.
     */
    Map(int width, int height, int cellSize);

    /**
     * @brief Initialize the grid as all free space, and marks the start location of the robot.
     */
    void initializeGrid();

    /**
     * @brief Updates the grid map with obstacle and free space information based on sensor readings.
     *
     * @param coords The current coordinates of the robot.
     * @param dLF Distance measured by the left front sensor.
     * @param dLB Distance measured by the left back sensor.
     * @param dRF Distance measured by the right front sensor.
     * @param dRB Distance measured by the right back sensor.
     * @param dFront Distance measured by the front sensor.
     * @param dBack Distance measured by the back sensor.
     */
    void updateGrid(float dLF, float dLB, float dRF, float dRB, float dFront, float dBack);

    void setRobotPosition(uint8_t x, uint8_t y);

    void turnRobotLeft(float angle);

    void turnRobotRight(float angle);

    void moveRobotForward(uint8_t distance);

    /**
     * @brief Print the grid map to the debug console, for debugging purposes.
     */
    void printGrid();

private:
    /**
     * @brief Calculates the global position of a sensor reading based on the robot's coordinates and orientation.
     *
     * @param coords The current coordinates and orientation of the robot.
     * @param offsetX The x-offset of the sensor from the robot's center.
     * @param offsetY The y-offset of the sensor from the robot's center.
     * @param distance The distance measured by the sensor.
     * @param sensorOrientation The orientation of the sensor relative to the robot.
     * @return std::pair<int, int> The calculated global coordinates.
     */
    std::pair<int, int> calculateGlobalPosition(float offsetX, float offsetY, float distance, float sensorOrientation);

    /**
     * @brief Marks the path from the robot's current location to the obstacle as free space.
     *
     * @param startX The starting x-coordinate (robot's location).
     * @param startY The starting y-coordinate (robot's location).
     * @param endX The ending x-coordinate (obstacle location).
     * @param endY The ending y-coordinate (obstacle location).
     */
    void markPath(int startX, int startY, int endX, int endY);

    /**
     * @brief Updates the grid with obstacle information based on a single sensor reading.
     *
     * @param coords The current coordinates of the robot.
     * @param sensorX The x-offset of the sensor from the robot's center.
     * @param sensorY The y-offset of the sensor from the robot's center.
     * @param distance The distance measured by the sensor.
     * @param sensorOrientation The orientation of the sensor relative to the robot.
     */
    void updateGridWithSensor(float sensorX, float sensorY, float distance, float sensorOrientation);

    /*
     * Grid definition variables
     */
    uint8_t width;      // Number of grid cells along the x-axis
    uint8_t height;     // Number of grid cells along the y-axis
    uint8_t cellSize;   // Dimension of each grid cell in centimeters
    uint8_t **grid; // 2D array representing the status of each cell in the grid map
    float robotOrientation; // the orientation in the robot 0-360 angle;
    uint8_t robotX, robotY;

    /*
     * Sensor locations relative to the center of the robot
     */
    const float sensorLF_X = -7.75;   // Left front sensor X offset
    const float sensorLF_Y = 10.0;    // Left front sensor Y offset
    const float sensorLB_X = -7.75;   // Left back sensor X offset
    const float sensorLB_Y = -10.0;   // Left back sensor Y offset
    const float sensorRF_X = 7.75;    // Right front sensor X offset
    const float sensorRF_Y = 10.0;    // Right front sensor Y offset
    const float sensorRB_X = 7.75;    // Right back sensor X offset
    const float sensorRB_Y = -10.0;   // Right back sensor Y offset
    const float sensorFront_X = 0.0;  // Front sensor X offset
    const float sensorFront_Y = 10.0; // Front sensor Y offset
    const float sensorBack_X = 0.0;   // Back sensor X offset
    const float sensorBack_Y = -10.0; // Back sensor Y offset

    /*
     * Constants for marking different elements on the grid
     */
    const int FREE_SPACE = 0;
    const int OBSTACLE = 1;
    const int ROBOT_LOCATION = 2;
};

#endif // MAP_H
