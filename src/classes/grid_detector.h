#ifndef GREEN_FUNDAMENTALS_GRID_DETECTOR_H
#define GREEN_FUNDAMENTALS_GRID_DETECTOR_H

#include "ros/ros.h"
#include <cstdlib>
#include "sensor_msgs/LaserScan.h"
#include <math.h>
#include "line.h"

class GridDetector {
private:
    float scalar_epsilon = 0.95;
    int min_matches = 5;
    int max_iteration = 1000;
    int leftover_amount = 5;

public:
    GridDetector();
    void detect_grid(const sensor_msgs::LaserScan::ConstPtr& laser_scan);

private:
    std::vector<Line> find_lines(std::vector<Point> points);
    std::vector<Point> get_cartesian_points(const sensor_msgs::LaserScan::ConstPtr& laser_scan);
    float get_distance_to_line(Line line);
};

#endif //GREEN_FUNDAMENTALS_GRID_DETECTOR_H