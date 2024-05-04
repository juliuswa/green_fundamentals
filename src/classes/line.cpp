//
// Created by laura on 03.05.24.
//

#include "line.h"

bool Line::get_distance_to_point(Vector point, float accuracy) {
    Vector point_offset(point.x - m_offset.x, point.y - m_offset.y);

    return point_offset.x * m_direction.y - point_offset.y * m_direction.x;
}

Line::Line(Vector direction, Vector offset) {
    float norm = std::sqrt(pow(direction.x, 2) + pow(direction.y, 2));
    
    Vector normed_direction(direction.x / norm, direction.y / norm);
    m_direction = normed_direction;

    Vector new_offset(offset.x, offset.y);
    m_offset = new_offset;
}