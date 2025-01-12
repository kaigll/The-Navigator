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

void Map::setCell(int x, int y, uint8_t state) {
    int index = (y * (width + 1) / 2) + (x / 2);
    if (x % 2 == 0) {
        grid[index] = (grid[index] & 0xF0) | (state & 0x0F); // Set lower 4 bits
    } else {
        grid[index] = (grid[index] & 0x0F) | ((state & 0x0F) << 4); // Set upper 4 bits
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

void Map::identifyStartPosition(float distanceBack, float distanceRightBack) {
    robotX = distanceRightBack + sensorRightBack_X;
    robotY = distanceBack - sensorBack_Y;

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

std::pair<int, int> Map::calculateGlobalPosition(float offsetX, float offsetY, float distance, float relativeAngle) {
    float combinedAngle = fmod((robotAngle + relativeAngle), 360.0f);
    float translatedX = (offsetX * cos(DEG_TO_RAD * robotAngle)) + (distance * cos(DEG_TO_RAD * combinedAngle)); // Calculate X translation
    float translatedY = (offsetY * sin(DEG_TO_RAD * robotAngle)) + (distance * sin(DEG_TO_RAD * combinedAngle)); // Calculate Y translation
    int globalX = static_cast<int>((robotX + translatedX) / cellSize);                                           // Convert to grid cell
    int globalY = static_cast<int>((robotY + translatedY) / cellSize);                                           // Convert to grid cell
    // Serial.println((String)globalX + ", " + globalY);

    return std::make_pair(globalX, globalY);
}

// potentially a deprecated function
void Map::updateGridWithSensor(float sensorX, float sensorY, float distance, float sensorAngle) {
    std::pair<int, int> globalPosition = calculateGlobalPosition(sensorX / cellSize, sensorY / cellSize, distance, sensorAngle);
    int cellX = globalPosition.first;
    int cellY = globalPosition.second;
    if (cellX >= 0 && cellX < width && cellY >= 0 && cellY < height) {
        markPath(robotX, robotY, cellX, cellY, FREE_SPACE); // Mark path to the obstacle
        setCell(cellX, cellY, OBSTACLE);                    // Mark the obstacle itself
    }
}

void Map::updateGrid(float distanceLeftFront, float distanceLeftBack, float distanceRightFront, float distanceRightBack, float distanceFront, float distanceBack) {
    std::pair<int, int> distanceFrontPosition = calculateGlobalPosition(sensorFront_X, sensorFront_Y, distanceFront, 0);
    // std::pair<int, int> distanceBackPosition = calculateGlobalPosition(sensorBack_X, sensorB_Y, distanceBack, 180);
    std::pair<int, int> distanceLeftFrontPosition = calculateGlobalPosition(sensorLeftFront_X, sensorLeftFront_Y, distanceLeftFront, 270);
    std::pair<int, int> distanceLeftBackPosition = calculateGlobalPosition(sensorLeftBack_X, sensorLeftBack_Y, distanceLeftBack, 270);
    std::pair<int, int> distanceRightFrontPosition = calculateGlobalPosition(sensorRightFront_X, sensorRightFront_Y, distanceRightFront, 90);
    std::pair<int, int> distanceRightBackPosition = calculateGlobalPosition(sensorRightBack_X, sensorRightBack_Y, distanceRightBack, 90);

    if (lastDistanceLeftFront = 0)
        lastDistanceLeftFront = distanceLeftFront;
    if (lastDistanceLeftBack = 0)
        lastDistanceLeftBack = distanceLeftBack;
    if (lastDistanceRightFront = 0)
        lastDistanceRightFront = distanceRightFront;
    if (lastDistanceRightBack = 0)
        lastDistanceRightBack = distanceRightBack;
    if (lastDistanceFront = 0)
        lastDistanceFront = distanceFront;
    if (lastDistanceBack = 0)
        lastDistanceBack = distanceBack;

    // check with a margain of error the difference between current and previous messurements to connect what is predicted to be continuous walls
    float errorForWall = 2;

    // check average of 2 measurements to check for blockage
    if (((distanceFront + lastDistanceFront) * 0.5) < BLOCKED_MARGAIN)
        _isFrontBlocked = true;
    else
        _isFrontBlocked = false;
    if (((distanceLeftFront + lastDistanceLeftFront) * 0.5) < BLOCKED_MARGAIN && ((distanceLeftBack + lastDistanceLeftBack) * 0.5) < BLOCKED_MARGAIN)
        _isLeftBlocked = true;
    else
        _isLeftBlocked = false;
    if (((distanceRightFront + lastDistanceRightFront) * 0.5) < BLOCKED_MARGAIN && ((distanceRightBack + lastDistanceRightBack) * 0.5) < BLOCKED_MARGAIN)
        _isRightBlocked = true;
    else
        _isRightBlocked = false;

    // check end of right side wall has not been reached
    if (!(fabs(lastDistanceLeftFront - distanceLeftFront) > errorForWall && fabs(distanceLeftFront - distanceLeftBack) > errorForWall)) {
        markPath(distanceLeftFrontPosition.first, distanceLeftFrontPosition.second, lastDistanceLeftFrontPosition.first, lastDistanceLeftFrontPosition.second, OBSTACLE);
    }
    // check end of right side wall has not been reached
    if (!(fabs(lastDistanceRightFront - distanceRightFront) > errorForWall && fabs(distanceRightFront - distanceRightBack) > errorForWall)) {
        markPath(distanceRightFrontPosition.first, distanceRightFrontPosition.second, lastDistanceRightFrontPosition.first, lastDistanceRightFrontPosition.second, OBSTACLE);
    }
    // if front > (significant) back then you know the wall ends
    // if back > (significant) front then you know new wall begins

    setCell(robotX, robotY, ROBOT_LOCATION); // Mark the robot's current location

    // store previous values for next cycle
    lastDistanceLeftFront = distanceLeftFront;
    lastDistanceLeftBack = distanceLeftBack;
    lastDistanceRightFront = distanceRightFront;
    lastDistanceRightBack = distanceRightBack;
    lastDistanceFront = distanceFront;
    lastDistanceBack = distanceBack;
    lastDistanceFrontPosition = distanceFrontPosition;
    // lastDistanceBackPosition = distanceBackPosition;
    lastDistanceLeftFrontPosition = distanceLeftFrontPosition;
    lastDistanceLeftBackPosition = distanceLeftBackPosition;
    lastDistanceRightFrontPosition = distanceRightFrontPosition;
    lastDistanceRightBackPosition = distanceRightBackPosition;
}

void Map::updateGrid(float distanceLeftFront, float distanceLeftBack, float distanceRightFront, float distanceRightBack) {
    updateGridWithSensor(sensorLeftFront_X, sensorLeftFront_Y, distanceLeftFront, 270);
    updateGridWithSensor(sensorLeftBack_X, sensorLeftBack_Y, distanceLeftBack, 270);
    updateGridWithSensor(sensorRightFront_X, sensorRightFront_Y, distanceRightFront, 90);
    updateGridWithSensor(sensorRightBack_X, sensorRightBack_Y, distanceRightBack, 90);

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

bool Map::isLeftBlocked() {
    return _isLeftBlocked;
}

bool Map::isRightBlocked() {
    return _isRightBlocked;
}

bool Map::isFrontBlocked() {
    return _isFrontBlocked;
}

bool Map::isRobotAtFinish() {
    // if robot is in the lower 15% of the maze, it has reached the finishing box
    if (robotY > height * 0.85) {
        digitalWrite(LEDR, LOW); // Turn on the red LED to show it knows where it is
        Serial.println("Finishing box has been reached");
        return true;
    } else {
        return false;
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
