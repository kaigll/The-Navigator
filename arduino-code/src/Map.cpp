#include "Map.h"

Map::Map(int w, int h, int cSize) : width(w), height(h), cellSize(cSize), robotX(0), robotY(0), robotAngle(90) {
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
    setCell(robotX, robotY, SEEN_LOCATION);

    // Set the new robot position
    robotX = x;
    robotY = y;
    robotAngle = angle % 360; // Ensure angle is within 0-359
    setCell(robotX, robotY, ROBOT_LOCATION);
}

void Map::identifyStartPosition(float dB, float dRB) {
    robotX = dRB + sensorRB_X;
    robotY = dB - sensorB_Y;

    setCell(robotX, robotY, ROBOT_LOCATION);
}

void Map::moveRobotForward(int distance) {
    distance = distance / cellSize;
    // Calculate the new position in one go
    int deltaX = round(cos(radians(robotAngle)) * distance);
    int deltaY = round(sin(radians(robotAngle)) * distance);
    int newX = robotX + deltaX;
    int newY = robotY + deltaY;

    markPath(robotX, robotY, newX, newY, SEEN_LOCATION);

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
}

void Map::rotateRobotRight(int degrees) {
    robotAngle = (robotAngle + degrees) % 360; // Rotate clockwise
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
    float combinedAngle = fmod((robotAngle + sensorAngle), 360.0f);
    float translatedX = (offsetX * cos(DEG_TO_RAD * robotAngle)) + (distance * cos(DEG_TO_RAD * combinedAngle)); // Calculate X translation
    float translatedY = (offsetY * sin(DEG_TO_RAD * robotAngle)) + (distance * sin(DEG_TO_RAD * combinedAngle)); // Calculate Y translation
    int globalX = static_cast<int>((robotX + translatedX) / cellSize);                                           // Convert to grid cell
    int globalY = static_cast<int>((robotY + translatedY) / cellSize);                                           // Convert to grid cell
    Serial.println((String)globalX + ", " + globalY);

    return std::make_pair(globalX, globalY);
}

void Map::updateGridWithSensor(float sensorX, float sensorY, float distance, float sensorAngle) {
    std::pair<int, int> globalPosition = calculateGlobalPosition(sensorX / cellSize, sensorY / cellSize, distance, sensorAngle);
    int cellX = globalPosition.first;
    int cellY = globalPosition.second;
    if (cellX >= 0 && cellX < width && cellY >= 0 && cellY < height) {
        markPath(robotX, robotY, cellX, cellY, FREE_SPACE); // Mark path to the obstacle
        setCell(cellX, cellY, OBSTACLE);                    // Mark the obstacle itself
    }
}

void Map::updateGrid(float dLF, float dLB, float dRF, float dRB, float dF, float dB) {
    if(!(dLF_prev == 0 || dLB_prev == 0 || dRF_prev == 0 || dRB_prev == 0 || dF_prev == 0 || dB_prev == 0)) {
        // CONTINUE ....
    }


    updateGridWithSensor(sensorF_X, sensorF_Y, dF, 0);
    updateGridWithSensor(sensorB_X, sensorB_Y, dB, 180);
    updateGridWithSensor(sensorLF_X, sensorLF_Y, dLF, 270);
    updateGridWithSensor(sensorLB_X, sensorLB_Y, dLB, 270);
    updateGridWithSensor(sensorRF_X, sensorRF_Y, dRF, 90);
    updateGridWithSensor(sensorRB_X, sensorRB_Y, dRB, 90);

    // if front > (significant) back then you know the wall ends
    // if back > (significant) front then you know new wall begins

    setCell(robotX, robotY, ROBOT_LOCATION); // Mark the robot's current location

    dLF_prev = dLF, dLB_prev = dLB, dRF_prev = dRF, dRB_prev = dRB, dF_prev = dF, dB_prev = dB;
}

void Map::updateGrid(float dLF, float dLB, float dRF, float dRB) {
    updateGridWithSensor(sensorLF_X, sensorLF_Y, dLF, 270);
    updateGridWithSensor(sensorLB_X, sensorLB_Y, dLB, 270);
    updateGridWithSensor(sensorRF_X, sensorRF_Y, dRF, 90);
    updateGridWithSensor(sensorRB_X, sensorRB_Y, dRB, 90);

    setCell(robotX, robotY, ROBOT_LOCATION); // Mark the robot's current location
}

void Map::markPath(int x1, int y1, int x2, int y2, uint8_t fillType) {
    // Bresenham's Line Algorithm
    int dx = abs(x2 - x1);
    int sx = (x1 < x2) ? 1 : -1;
    int dy = abs(y2 - y1);
    dy = -dy;
    int sy = (y1 < y2) ? 1 : -1;
    int err = dx + dy;

    while (true) {
        if (getCell(x1, y1) != ROBOT_LOCATION || getCell(x1, y1) != SEEN_LOCATION) {
            setCell(x1, y1, fillType);
        }
        if ((x1 == x2) && (y1 == y2))
            break;
        int e2 = err << 1;
        if (e2 >= dy) {
            if (x1 == x2)
                break;
            err += dy;
            x1 += sx;
        }
        if (e2 <= dx) {
            if (y1 == y2)
                break;
            err += dx;
            y1 += sy;
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
            Serial.print("");
        }
        Serial.println();
    }
}
