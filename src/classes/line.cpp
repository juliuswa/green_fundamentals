#include "line.h"

float Line::get_distance_from_point(Eigen::Vector2f point) {
    return std::abs((point[0] - m_offset[0]) * m_direction[1] - (point[1] - m_offset[1]) * m_direction[0]);
}

Eigen::Vector2f Line::get_projection_of_point(Eigen::Vector2f point) {
    Eigen::Vector2f delta = point - m_offset;
    
    float nominator = delta[0] * m_direction[0] + delta[1] * m_direction[1];
    float denominator = m_direction[0] * m_direction[0] + m_direction[1] * m_direction[1];

    float lambda = nominator / denominator;

    return m_offset + (m_direction * lambda);
}

Eigen::Vector2f Line::get_polar_representation() {
    Eigen::Vector2f polar_representation;

    Eigen::Vector2f reference_position {0.0, 0.0};
    float distance = get_distance_from_point(reference_position);
    polar_representation[0] = distance;

    Eigen::Vector2f position_projection = get_projection_of_point(reference_position);
    Eigen::Vector2f reference_direction {1.0, 0.0};

    float scalar_product = position_projection[0] * reference_direction[0] + position_projection[1] * reference_direction[1];
    polar_representation[1] = acos(scalar_product / (position_projection.norm() * reference_direction.norm()));

    float cross_product_z = position_projection[0] * reference_direction[1] - position_projection[1] * reference_direction[0];

    if (cross_product_z > 0) {
        polar_representation[1] = polar_representation[1]* -1;
    }  

    return polar_representation;
}

Eigen::Vector2f Line::get_cut_vertex(Line other_line) {
  Eigen::Matrix2f A;
  A << m_direction, other_line.m_direction;
  Eigen::Vector2f b;
  b << other_line.m_offset.dot(m_direction) - m_offset.dot(m_direction);

  Eigen::Vector2f x = A.inverse() * b;
  
  return x;
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