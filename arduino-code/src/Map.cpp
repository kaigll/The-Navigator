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
    // Calculate the new position in one go
    int deltaX = round(cos(radians(robotAngle)) * distance / cellSize);
    int deltaY = round(sin(radians(robotAngle)) * distance / cellSize);
    int newX = robotX + deltaX;
    int newY = robotY + deltaY;

    // Check for out-of-bounds or obstacles in the path
    if (newX < 0 || newX >= width || newY < 0 || newY >= height || getCell(newX, newY) == OBSTACLE) {
        Serial.println("Obstacle encountered or out of bounds!");
        return;
    }

    // Set the new robot position
    setRobotPosition(newX, newY, robotAngle);
}

void Map::rotateRobotLeft(int degrees) {
    robotAngle = (robotAngle + 360 - degrees) % 360; // Rotate anticlockwise
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

std::pair<int, int> Map::calculateGlobalPosition(float offsetX, float offsetY, float distance, float sensorAngle) {
    float rad = fmod((robotAngle + sensorAngle), 360.0f) * (PI / 180.0); // Convert orientation to radians
    int globalX = robotX + static_cast<int>((offsetX + distance * -sin(rad)) / cellSize);
    int globalY = robotY + static_cast<int>((offsetY + distance * cos(rad)) / cellSize);
    Serial.println((String)globalX + ", " + globalY);

    return std::make_pair(globalX, globalY);
}

void Map::updateGridWithSensor(float sensorX, float sensorY, float distance, float sensorAngle) {
    std::pair<int, int> globalPosition = calculateGlobalPosition(sensorX / cellSize, sensorY / cellSize, distance, sensorAngle);
    int cellX = globalPosition.first;
    int cellY = globalPosition.second;
    if (cellX >= 0 && cellX < width && cellY >= 0 && cellY < height) {
        markPath(robotX, robotY, cellX, cellY); // Mark path to the obstacle
        setCell(cellX, cellY, OBSTACLE);        // Mark the obstacle itself
    }
}

void Map::updateGrid(float dLF, float dLB, float dRF, float dRB, float dF, float dB) {
    updateGridWithSensor(sensorF_X, sensorF_Y, dF, 0);
    updateGridWithSensor(sensorB_X, sensorB_Y, dB, 180);
    updateGridWithSensor(sensorLF_X, sensorLF_Y, dLF, 270);
    updateGridWithSensor(sensorLB_X, sensorLB_Y, dLB, 270);
    updateGridWithSensor(sensorRF_X, sensorRF_Y, dRF, 90);
    updateGridWithSensor(sensorRB_X, sensorRB_Y, dRB, 90);

    setCell(robotX, robotY, ROBOT_LOCATION); // Mark the robot's current location
}

void Map::updateGrid(float dLF, float dLB, float dRF, float dRB) {
    updateGridWithSensor(sensorLF_X, sensorLF_Y, dLF, 270);
    updateGridWithSensor(sensorLB_X, sensorLB_Y, dLB, 270);
    updateGridWithSensor(sensorRF_X, sensorRF_Y, dRF, 90);
    updateGridWithSensor(sensorRB_X, sensorRB_Y, dRB, 90);

    setCell(robotX, robotY, ROBOT_LOCATION); // Mark the robot's current location
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
            setCell(x, y, FREE_SPACE);
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

// Debugging method
void Map::printGrid() {
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            if (getCell(x, y) == FREE_SPACE) {
                Serial.print(".");
            } else {
                Serial.print(getCell(x, y));
            }
            Serial.print(" ");
        }
        Serial.println();
    }
}
