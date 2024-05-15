#include "line.h"

float Line::get_distance_to_point(Eigen::Vector2f point) {
    return (point[0] - m_offset[0]) * m_direction[1] - (point[1] - m_offset[1]) * m_direction[0];
}

Eigen::Vector2f Line::get_polar_representation() {
    Eigen::Vector2f polar_representation;

    Eigen::Vector2f reference_position {0.0, 0.0};
    polar_representation[0] = get_distance_to_point(reference_position);

    Eigen::Vector2f reference_direction {1.0, 0.0};

    float scalar_product = m_direction[0] * reference_direction[0] + m_direction[1] * reference_direction[1];
    polar_representation[1] = acos(scalar_product / (m_direction.norm() * reference_direction.norm()));

    return polar_representation;
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