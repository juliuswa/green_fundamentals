#ifndef GREEN_FUNDAMENTALS_GRID_DETECTOR_H
#define GREEN_FUNDAMENTALS_GRID_DETECTOR_H

#include "ros/ros.h"
#include <cstdlib>
#include "sensor_msgs/LaserScan.h"
#include <math.h>
#include <list>
#include <set>
#include <chrono>
#include "../algorithms/ransack.cpp"

class LineDetector {
private:
    const float theta_offset = 0.74; 
    const float epsilon = 0.05;
    const int min_matches = 10;

    float m_last_measurement[1000];

public:
    LineDetector();
    void detect(const sensor_msgs::LaserScan::ConstPtr& laser_scan);

private:
    std::list<Vector> get_measurements(const sensor_msgs::LaserScan::ConstPtr& laser_scan);
    std::list<Line> find_lines(std::list<Vector> measurements);
    float get_distance_to_line(Line line, float accuracy);
};

#endif //GREEN_FUNDAMENTALS_GRID_DETECTOR_H