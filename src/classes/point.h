#ifndef GREEN_FUNDAMENTALS_POINT_H
#define GREEN_FUNDAMENTALS_POINT_H

#include <cmath>

class Point {
public:
    float x;
    float y;

    float r;
    float theta;

    Point(float a, float b, bool is_polar);
};


#endif //GREEN_FUNDAMENTALS_POINT_H
