#ifndef MAP_H
#define MAP_H

#include <Arduino.h>
#include <cmath>
#include <cstdint>

// --- Constants representing the state different points on the map ---

const uint8_t FREE_SPACE = 0;
const uint8_t OBSTACLE = 1;
const uint8_t SEEN_LOCATION = 2;
const uint8_t ROBOT_LOCATION = 3;

class Map {
public:
    Map(int width, int height, int cellSize);
    ~Map();

    /**
     * @brief Set the state of a cell
     *
     * @param x coordinate
     * @param y coordinate
     * @param state the current state of the cell at (x, y)
     */
    void setCell(int x, int y, uint8_t state);

    /**
     * @brief Returns the state of the cell located at a given (x, y) corrdinate.
     *
     * @param x coordinate
     * @param y coordinate
     */
    uint8_t getCell(int x, int y);

    /**
     * @brief Debugging method: Prints the full grid to Serial. This method is intensive on resources due to recurrsion (O(n^2) time complexity)
     * therefore it is best to not call frequently.
     */
    void printGrid();

    // --- Robot position related methods ---
    /**
     * @brief identifies the starting position of the robot based upon being positioned in the top left of the maze. Facing down the y axis
     *
     * @param x grid x position
     * @param y grid y position
     * @param angle in degrees (0-359)
     */
    void setRobotPosition(int x, int y, int angle);

    /**
     * @brief identifies the starting position of the robot based upon being positioned in the top left of the maze. Facing down the y axis
     *
     * @param distanceBackR distance from back right sensor
     * @param distanceBack distance from back sensor
     */
    void identifyStartPosition(float distanceBack, float distanceBackR);

    void moveRobotForward(int distance);
    void rotateRobotLeft(int degrees);
    void rotateRobotRight(int degrees);

    // --- Getters, these are private variables that shouldn't be directly accessed ---

    int getRobotX();
    int getRobotY();
    int getRobotAngle();
    bool isLeftBlocked();
    bool isRightBlocked();
    bool isFrontBlocked();

    // --- Grid updating, and position mapping functions ---

    std::pair<int, int> calculateGlobalPosition(float offsetX, float offsetY, float distance, float sensorAngle);
    void updateGridWithSensor(float sensorX, float sensorY, float distance, float sensorAngle);
    void updateGrid(float distanceLeftFront, float distanceLeftBack, float distanceRightFront, float distanceRightBack, float distanceFront, float distanceBack);
    void updateGrid(float distanceLeftFront, float distanceLeftBack, float distanceRightFront, float distanceRightBack);

    bool isRobotAtFinish();

private:
    uint8_t *grid; // The full grid stored as a one-dimensional array
    int width;
    int height;
    int cellSize;
    int robotX;
    int robotY;
    int robotAngle; // Angle in degrees (0 - 359)

    /**
     * @brief Initialise the grid in memory. The grid is using height*(width/2) 8 bit unsigned integars in memory,
     * due to each grid cell only requiring a nibble to store the state, each index of the array stores 2 values.
     *
     * This was done in an attempt to reduce the map size in memory as much as possible as I suspected a full 32 bit
     * integer was causing issues with Serial communication when printing the grid.
     */
    void initGrid();

    void markPath(int x0, int y0, int x1, int y1, uint8_t type);

    // Booleans for checking if a side is blocked

    bool _isLeftBlocked = false;
    bool _isRightBlocked = false;
    bool _isFrontBlocked = false;
    const int BLOCKED_MARGAIN = 5;

    // Saved values of the previous sensor measurements

    float lastDistanceLeftFront, lastDistanceLeftBack, lastDistanceRightFront, lastDistanceRightBack, lastDistanceFront, lastDistanceBack = 0;
    std::pair<int, int> lastDistanceLeftFrontPosition, lastDistanceLeftBackPosition, lastDistanceRightFrontPosition, lastDistanceRightBackPosition,
        lastDistanceFrontPosition, lastDistanceBackPosition = std::make_pair(0, 0);

    // Sensor locations relative to the center of the robot

    const float sensorLeftFront_X = -7.75; // Left front sensor X offset
    const float sensorLeftFront_Y = 10.0;  // Left front sensor Y offset
    const float sensorLeftBack_X = -7.75;  // Left back sensor X offset
    const float sensorLeftBack_Y = -10.0;  // Left back sensor Y offset
    const float sensorRightFront_X = 7.75; // Right front sensor X offset
    const float sensorRightFront_Y = 10.0; // Right front sensor Y offset
    const float sensorRightBack_X = 7.75;  // Right back sensor X offset
    const float sensorRightBack_Y = -10.0; // Right back sensor Y offset
    const float sensorFront_X = 0.0;       // Front sensor X offset
    const float sensorFront_Y = 10.0;      // Front sensor Y offset
    const float sensorBack_X = 0.0;        // Back sensor X offset
    const float sensorBack_Y = -10.0;      // Back sensor Y offset
};

#endif // MAP_H
