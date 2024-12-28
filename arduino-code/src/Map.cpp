#include "Map.h"

Map::Map(int w, int h, int cSize) : width(w), height(h), cellSize(cSize), robotX(0), robotY(0), robotAngle(0) {
    initGrid();
}

Map::~Map() {
    delete[] grid;
}

void Map::initGrid() {
    int byteWidth = (width + 1) / 2; // Width in bytes, each byte stores two cells
    grid = new uint8_t[height * byteWidth]();
}

void Map::setCell(int x, int y, uint8_t value) {
    int index = (y * (width + 1) / 2) + (x / 2);
    if (x % 2 == 0) {
        grid[index] = (grid[index] & 0xF0) | (value & 0x0F); // Set lower 4 bits
    } else {
        grid[index] = (grid[index] & 0x0F) | ((value & 0x0F) << 4); // Set upper 4 bits
    }
}

uint8_t Map::getCell(int x, int y) {
    int index = (y * (width + 1) / 2) + (x / 2);
    if (x % 2 == 0) {
        return grid[index] & 0x0F; // Get lower 4 bits
    } else {
        return (grid[index] >> 4) & 0x0F; // Get upper 4 bits
    }
}

void Map::setRobotPosition(int x, int y, int angle) {
    // Clear the current robot position
    setCell(robotX, robotY, FREE_SPACE);

    // Set the new robot position
    robotX = x;
    robotY = y;
    robotAngle = angle % 360; // Ensure angle is within 0-359
    setCell(robotX, robotY, ROBOT_LOCATION);
}

void Map::moveRobotForward(int distance) {
    // Move the robot forward by distance cells in the current direction
    for (int i = 0; i < distance; ++i) {
        int newX = robotX + round(cos(radians(robotAngle)) * cellSize);
        int newY = robotY + round(sin(radians(robotAngle)) * cellSize);
        if (newX < 0 || newX >= width || newY < 0 || newY >= height || getCell(newX, newY) == OBSTACLE) {
            Serial.println("Obstacle encountered or out of bounds!");
            break;
        }
        setRobotPosition(newX, newY, robotAngle);
    }
}

void Map::rotateRobotLeft(int degrees) {
    robotAngle = (robotAngle + 360 - degrees) % 360; // Rotate counterclockwise
    setCell(robotX, robotY, ROBOT_LOCATION);         // Update robot position with the new direction
}

void Map::rotateRobotRight(int degrees) {
    robotAngle = (robotAngle + degrees) % 360; // Rotate clockwise
    setCell(robotX, robotY, ROBOT_LOCATION);   // Update robot position with the new direction
}

int Map::getRobotX() {
    return robotX;
}

int Map::getRobotY() {
    return robotY;
}

int Map::getRobotAngle() {
    return robotAngle;
}

void Map::updateGridWithSensorData(float distance, int sensorOffsetX, int sensorOffsetY, int sensorAngle) {
    // Calculate the sensor's position relative to the robot
    int sensorX = robotX + round(cos(radians(robotAngle)) * sensorOffsetX - sin(radians(robotAngle)) * sensorOffsetY);
    int sensorY = robotY + round(sin(radians(robotAngle)) * sensorOffsetX + cos(radians(robotAngle)) * sensorOffsetY);
    // Calculate the endpoint based on the sensor angle and distance
    int endX = sensorX + round(cos(radians(sensorAngle)) * distance / cellSize);
    int endY = sensorY + round(sin(radians(sensorAngle)) * distance / cellSize);
    // Mark the line from the sensor position to the endpoint as FREE_SPACE
    markPath(sensorX, sensorY, endX, endY);
}

void Map::markPath(int x0, int y0, int x1, int y1) {
    // Use Bresenham's Line Algorithm to mark all cells along the line
    int dx = abs(x1 - x0);
    int dy = abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;
    while (true) {
        // Set cells along the line as FREE_SPACE
        setCell(x0, y0, FREE_SPACE);
        if (x0 == x1 && y0 == y1)
            break;
        int e2 = err * 2;
        if (e2 > -dy) {
            err -= dy;
            x0 += sx;
        }
        if (e2 < dx) {
            err += dx;
            y0 += sy;
        }
    }
    // Set the endpoint as OBSTACLE
    setCell(x1, y1, OBSTACLE);
}

// Debugging method
void Map::printGrid() {
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            Serial.print(getCell(x, y));
            Serial.print(" ");
        }
        Serial.println();
    }
}
