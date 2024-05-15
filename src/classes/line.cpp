#include "line.h"

float Line::get_distance_to_point(Eigen::Vector2f point) {
    return (point[0] - m_offset[0]) * m_direction[1] - (point[1] - m_offset[1]) * m_direction[0];
}

Line::Line(Eigen::Vector2f direction, Eigen::Vector2f offset) {
    m_direction = direction / direction.norm();
    m_offset = offset;
    m_score = 0;
}

Line::Line() {
    m_offset[0] = 0.0f;
    m_offset[1] = 0.0f;

    m_direction[0] = 0.0f;
    m_direction[1] = 0.0f;

    m_score = 0;
}