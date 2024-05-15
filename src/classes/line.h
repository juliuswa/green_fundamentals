#ifndef GREEN_FUNDAMENTALS_LINE_H
#define GREEN_FUNDAMENTALS_LINE_H

#include "ros/ros.h"
#include "vector.h"
#include "../Eigen/Dense"

class Line {
public:
    Eigen::Vector2f m_direction;
    Eigen::Vector2f m_offset;
    int m_score;

    Line();
    Line(Eigen::Vector2f direction, Eigen::Vector2f offset);
    float get_distance_to_point(Eigen::Vector2f point);
    Eigen::Vector2f get_polar_representation();
};

#endif //GREEN_FUNDAMENTALS_LINE_H
