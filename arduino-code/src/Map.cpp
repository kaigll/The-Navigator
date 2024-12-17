#include "Map.h"
#include <Arduino.h>

Map::Map(int width, int height, int cellSize) : width(width), height(height), cellSize(cellSize) {
    grid = new int *[width];
    for (int i = 0; i < width; ++i) {
        grid[i] = new int[height];
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

void Map::updateGrid(Coordinate coords, float dLF, float dLB, float dRF, float dRB, float dFront, float dBack) {
    int cellX, cellY;

    auto calculateGlobalPosition = [this, &coords](float offsetX, float offsetY, float distance, float sensorOrientation) {
        float rad = fmod((coords.getOrientation() + sensorOrientation), 360.0f) * (PI / 180.0); // Convert orientation to radians
        int globalX = coords.getX() + static_cast<int>((offsetX + distance * -sin(rad)) / cellSize);
        int globalY = coords.getY() + static_cast<int>((offsetY + distance * cos(rad)) / cellSize);
        Serial.print(globalX);
        Serial.print(" ,");
        Serial.println(globalY);

        
        // reference of center for coordinates
        return std::make_pair(globalX, globalY);
    };

    // Update grid based on left front sensor
    std::tie(cellX, cellY) = calculateGlobalPosition(sensorLF_X/cellSize, sensorLF_Y/cellSize, dLF, 270);
    if (cellX >= 0 && cellX < width && cellY >= 0 && cellY < height) {
        grid[cellX][cellY] = OBSTACLE;
    }

    // Update grid based on left back sensor
    std::tie(cellX, cellY) = calculateGlobalPosition(sensorLB_X/cellSize, sensorLB_Y/cellSize, dLB, 270);
    if (cellX >= 0 && cellX < width && cellY >= 0 && cellY < height) {
        grid[cellX][cellY] = OBSTACLE;
    }

    // Update grid based on right front sensor
    std::tie(cellX, cellY) = calculateGlobalPosition(sensorRF_X/cellSize, sensorRF_Y/cellSize, dRF, 90);
    if (cellX >= 0 && cellX < width && cellY >= 0 && cellY < height) {
        grid[cellX][cellY] = OBSTACLE;
    }

    // Update grid based on right back sensor
    std::tie(cellX, cellY) = calculateGlobalPosition(sensorRB_X/cellSize, sensorRB_Y/cellSize, dRB, 90);
    if (cellX >= 0 && cellX < width && cellY >= 0 && cellY < height) {
        grid[cellX][cellY] = OBSTACLE;
    }

    // Update grid based on front sensor
    std::tie(cellX, cellY) = calculateGlobalPosition(sensorFront_X/cellSize, sensorFront_Y/cellSize, dFront, 0);
    if (cellX >= 0 && cellX < width && cellY >= 0 && cellY < height) {
        grid[cellX][cellY] = OBSTACLE;
    }

    // Update grid based on back sensor
    std::tie(cellX, cellY) = calculateGlobalPosition(sensorBack_X/cellSize, sensorBack_Y/cellSize, dBack, 180);
    if (cellX >= 0 && cellX < width && cellY >= 0 && cellY < height) {
        grid[cellX][cellY] = OBSTACLE;
    }

    grid[coords.getX()][coords.getY()] = ROBOT_LOCATION;
}

// Optional: For debugging purposes
void Map::printGrid() {
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            Serial.print(grid[x][y]);
            Serial.print(" ");
        }
        Serial.println();
    }
}
