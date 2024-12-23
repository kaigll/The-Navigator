#include <Coordinate.h>
#include <cmath>

Coordinate::Coordinate(int x, int y, float orientation) : x(x), y(y), orientation(orientation) {}

void Coordinate::setX(int x) {
    this->x = x;
}

void Coordinate::setY(int y) {
    this->y = y;
}

void Coordinate::addX(int dx) {
    this->x += dx;
}

void Coordinate::addY(int dy) {
    this->y += dy;
}

void Coordinate::turnLeft(float angle) {
    orientation = fmod((orientation - angle + 360.0f), 360.0f);
}

void Coordinate::turnRight(float angle) {
    orientation = fmod((orientation + angle), 360.0f);
}

void Coordinate::moveForward(float distance) {
    float rad = fmod(orientation, 360.0f) * (M_PI / 180.0);
    x += static_cast<int>(distance * cos(rad));
    y += static_cast<int>(distance * sin(rad));
}

int Coordinate::getX() {
    return x;
}

int Coordinate::getY() {
    return y;
}

float Coordinate::getOrientation() {
    return orientation;
}