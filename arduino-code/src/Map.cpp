#include "Map.h"
#include <Arduino.h>

Map::Map(int width, int height, int cellSize) : width(width), height(height), cellSize(cellSize) {
    grid = new uint8_t *[width];
    for (int i = 0; i < width; ++i) {
        grid[i] = new uint8_t[height];
    }
    initializeGrid();
}

void Map::initializeGrid() {
    for (int x = 0; x < width; x++) {
        for (int y = 0; y < height; y++) {
            grid[x][y] = FREE_SPACE; // Initialize all cells as free space
        }
    }
}

std::pair<int, int> Map::calculateGlobalPosition(float offsetX, float offsetY, float distance, float sensorOrientation) {
    float rad = fmod((robotOrientation + sensorOrientation), 360.0f) * (PI / 180.0); // Convert orientation to radians
    int globalX = robotX + static_cast<int>((offsetX + distance * -sin(rad)) / cellSize);
    int globalY = robotY + static_cast<int>((offsetY + distance * cos(rad)) / cellSize);
    Serial.print(globalX);
    Serial.print(" ,");
    Serial.println(globalY);

    return std::make_pair(globalX, globalY);
}

void Map::markPath(int startX, int startY, int endX, int endY) {
    int x = startX;
    int y = startY;
    int dx = abs(endX - startX);
    int dy = abs(endY - startY);
    int sx = (startX < endX) ? 1 : -1;
    int sy = (startY < endY) ? 1 : -1;
    int err = dx - dy;

    while (true) {
        if (x >= 0 && x < width && y >= 0 && y < height) {
            grid[x][y] = FREE_SPACE;
        }
        if (x == endX && y == endY)
            break;
        int e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            x += sx;
        }
        if (e2 < dx) {
            err += dx;
            y += sy;
        }
    }
}

void Map::updateGridWithSensor(float sensorX, float sensorY, float distance, float sensorOrientation) {
    std::pair<int, int> globalPosition = calculateGlobalPosition(sensorX / cellSize, sensorY / cellSize, distance, sensorOrientation);
    int cellX = globalPosition.first;
    int cellY = globalPosition.second;
    if (cellX >= 0 && cellX < width && cellY >= 0 && cellY < height) {
        markPath(robotX, robotY, cellX, cellY); // Mark path to the obstacle
        grid[cellX][cellY] = OBSTACLE;                        // Mark the obstacle itself
    }
}

void Map::updateGrid(float dLF, float dLB, float dRF, float dRB, float dFront, float dBack) {
    updateGridWithSensor(sensorLF_X, sensorLF_Y, dLF, 270);
    updateGridWithSensor(sensorLB_X, sensorLB_Y, dLB, 270);
    updateGridWithSensor(sensorRF_X, sensorRF_Y, dRF, 90);
    updateGridWithSensor(sensorRB_X, sensorRB_Y, dRB, 90);
    updateGridWithSensor(sensorFront_X, sensorFront_Y, dFront, 0);
    updateGridWithSensor(sensorBack_X, sensorBack_Y, dBack, 180);

    grid[robotX][robotY] = ROBOT_LOCATION; // Mark the robot's current location
}

void Map::setRobotPosition(uint8_t x, uint8_t y) {
    robotX = x;
    robotY = y;
}

void Map::turnRobotLeft(float angle) {
    robotOrientation = fmod((robotOrientation - angle + 360.0f), 360.0f);
}

void Map::turnRobotRight(float angle) {
    robotOrientation = fmod((robotOrientation + angle), 360.0f);
}

void Map::moveRobotForward(uint8_t distance) {
    float rad = fmod(robotOrientation, 360.0f) * (M_PI / 180.0);
    robotX += static_cast<int>(distance * cos(rad));
    robotY += static_cast<int>(distance * sin(rad));
}

// Optional: For debugging purposes
void Map::printGrid() {
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            if (grid[x][y] == 0) {
                Serial.print(".");
            } else {
                Serial.print(grid[x][y]);
            }
            Serial.print(" ");
        }
        Serial.println();
    }
}
