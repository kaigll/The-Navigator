#ifndef MAP_H
#define MAP_H

#include <Coordinate.h>

class Map {
public:
    Map(int width, int height, int cellSize);
    void initializeGrid();
    void updateGrid(Coordinate coords, float dLF, float dLB, float dRF, float dRB, float dFront, float dBack);
    void printGrid(); // Optional: For debugging purposes

private:
    int width;
    int height;
    int cellSize;
    int **grid;

    const float sensorLF_X = -7.75; // Left front sensor X offset 
    const float sensorLF_Y = 10.0; // Left front sensor Y offset 
    const float sensorLB_X = -7.75; // Left back sensor X offset 
    const float sensorLB_Y = -10.0; // Left back sensor Y offset 
    const float sensorRF_X = 7.75; // Right front sensor X offset 
    const float sensorRF_Y = 10.0; // Right front sensor Y offset 
    const float sensorRB_X = 7.75; // Right back sensor X offset 
    const float sensorRB_Y = -10.0; // Right back sensor Y offset 
    const float sensorFront_X = 0.0; // Front sensor X offset 
    const float sensorFront_Y = 10.0; // Front sensor Y offset 
    const float sensorBack_X = 0.0; // Back sensor X offset 
    const float sensorBack_Y = -10.0; // Back sensor Y offset

    const int FREE_SPACE = 0;
    const int OBSTACLE = 1;
    const int ROBOT_LOCATION = 2;
};

#endif // MAP_H
