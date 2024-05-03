#ifndef GREEN_FUNDAMENTALS_GRID_DETECTOR_H
#define GREEN_FUNDAMENTALS_GRID_DETECTOR_H

#include "ros/ros.h"
#include <cstdlib>
#include "sensor_msgs/LaserScan.h"
#include <math.h>
#include <list>
#include <set>
#include "line.h"
#include "vector.h"

class GridDetector {
private:
    const float epsilon = 0.02;
    const int min_matches = 5;
    const int max_iteration = 1000;
    const int leftover_amount = 5;

public:
    GridDetector();
    void detect_grid(const sensor_msgs::LaserScan::ConstPtr& laser_scan);

private:
    std::list<Vector> get_measurements(const sensor_msgs::LaserScan::ConstPtr& laser_scan);
    std::list<Line> find_lines(std::list<Vector> measurements);
    float get_distance_to_line(Line line, float accuracy);
};

#endif //GREEN_FUNDAMENTALS_GRID_DETECTOR_H