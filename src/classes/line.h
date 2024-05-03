//
// Created by laura on 03.05.24.
//

#ifndef GREEN_FUNDAMENTALS_VECTOR_H
#define GREEN_FUNDAMENTALS_VECTOR_H

#include "point.h"

class Line {
public:
    float x;
    float y;
    Point offset;

private:
    Line(float unnormed_x, float unnormed_y, Point offset_point);
};


#endif //GREEN_FUNDAMENTALS_VECTOR_H
