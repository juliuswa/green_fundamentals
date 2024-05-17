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

struct GridDetectorResponse {
    bool success;
    Eigen::Vector2f offset;
    float theta;
};

class GridDetector {
private:
    const float theta_offset = - 2.37; 
    const float x_offset = 0.13;

    std::list<Eigen::Vector2f> m_current_measurement;
    float m_raw_last_measurement[2000];

    bool received_packet = false;

public:
    GridDetector();
    void get_measurements(const sensor_msgs::LaserScan::ConstPtr& laser_scan);
    GridDetectorResponse detect();

private:
    std::list<Line> find_lines(std::list<Vector> measurements);
    float get_distance_to_line(Line line, float accuracy);
    GridDetectorResponse create_grid_detector_response(std::vector<Line> lines);
};

#endif //GREEN_FUNDAMENTALS_GRID_DETECTOR_H