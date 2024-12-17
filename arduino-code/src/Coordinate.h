#ifndef COORDINATE_H
#define COORDINATE_H

class Coordinate {
public:
    Coordinate(int x, int y, float orientation);

    void setX(int x);
    void setY(int y);
    void addX(int dx);
    void addY(int dy);
    void turnLeft(float angle);
    void turnRight(float angle);

    int getX();
    int getY();
    float getOrientation();
private:
    int x;
    int y;
    float orientation;
};


#endif // COORDINATE_H
