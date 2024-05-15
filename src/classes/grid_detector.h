#ifndef GREEN_FUNDAMENTALS_GRID_DETECTOR_H
#define GREEN_FUNDAMENTALS_GRID_DETECTOR_H

#include "ros/ros.h"
#include <cstdlib>
#include "sensor_msgs/LaserScan.h"
#include <math.h>
#include <list>
#include <set>
#include <chrono>
#include "../algorithms/ransac.cpp"
#include "../algorithms/find_midpoint_from_lines.cpp"

class GridDetector {
private:
    const float theta_offset = - 2.37; 
    const float x_offset = 0.13;
    const float epsilon = 0.1;
    const int min_matches = 5;

    float m_last_measurement[1000];

    bool received_packet = false;

public:
    GridDetector();
    void detect(const sensor_msgs::LaserScan::ConstPtr& laser_scan);
    // std::pair<Vector, Vector> mid_and_ori = {Vector(FLT_MAX, FLT_MAX), Vector(FLT_MAX, FLT_MAX)};

private:
    std::list<Eigen::Vector2f> get_measurements(const sensor_msgs::LaserScan::ConstPtr& laser_scan);
    std::list<Line> find_lines(std::list<Vector> measurements);
    float get_distance_to_line(Line line, float accuracy);
    std::vector<Line> summarize_lines(std::vector<Line> lines);
};

#endif //GREEN_FUNDAMENTALS_GRID_DETECTOR_H