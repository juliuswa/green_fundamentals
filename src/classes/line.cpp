//
// Created by laura on 03.05.24.
//

#include "line.h"

bool Line::get_distance_to_point(Vector point, float accuracy) {
    int step_amount = std::round(1.0 / accuracy) * max_lenght;

    float min_distance = 1.0;

    for (int i = -step_amount; i < step_amount; i++) {
        float x_coordinate = m_offset.x + (i * m_direction.x / step_amount);
        float y_coordinate = m_offset.y + (i * m_direction.y / step_amount);

        float x_distance = x_coordinate - point.x;
        float y_distance = y_coordinate - point.y;

        float distance = std::sqrt(pow(x_distance, 2) + pow(y_distance, 2));

        if (distance < min_distance) {
            min_distance = distance;
        }
    }

    return min_distance;
}

Line::Line(Vector direction, Vector offset) {
    float norm = std::sqrt(pow(direction.x, 2) + pow(direction.y, 2));
    
    Vector normed_direction(direction.x / norm, direction.y / norm);
    m_direction = normed_direction;

    Vector new_offset(offset.x, offset.y);
    offset = new_offset;
}