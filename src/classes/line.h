#ifndef GREEN_FUNDAMENTALS_LINE_H
#define GREEN_FUNDAMENTALS_LINE_H

#include "vector.h"

class Line {
public:
    Vector m_direction;
    Vector m_offset;

    Line();
    Line(Vector direction, Vector offset);
    float get_distance_to_point(Vector point);
};


#endif //GREEN_FUNDAMENTALS_LINE_H
