#ifndef COORDINATE_H
#define COORDINATE_H

#include <cstdint>

class Coordinate {
public:
    Coordinate(int x, int y, float orientation);

    void setX(int x);
    void setY(int y);
    void addX(int dx);
    void addY(int dy);
    void turnLeft(float angle);
    void turnRight(float angle);
    void moveForward(float distance);

    int getX();
    int getY();
    float getOrientation();
private:
    uint8_t x;
    uint8_t y;
    float orientation;
};


#endif // COORDINATE_H
