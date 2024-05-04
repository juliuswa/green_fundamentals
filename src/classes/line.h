#ifndef GREEN_FUNDAMENTALS_LINE_H
#define GREEN_FUNDAMENTALS_LINE_H

#include "vector.h"
#include <string>

class Line {
public:
    float max_lenght = 1.0;
    Vector m_direction;
    Vector m_offset;

    Line(Vector direction, Vector offset);
    float get_distance_to_point(Vector point);
};


#endif //GREEN_FUNDAMENTALS_LINE_H
