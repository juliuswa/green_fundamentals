#ifndef GREEN_FUNDAMENTALS_GRID_DETECTOR_H
#define GREEN_FUNDAMENTALS_GRID_DETECTOR_H

#include "ros/ros.h"
#include <cstdlib>
#include "sensor_msgs/LaserScan.h"
#include <math.h>

class GridDetector {
private:

public:
    GridDetector();
    void detect_grid(const sensor_msgs::LaserScan::ConstPtr& laser_scan);

private:

};

#endif //GREEN_FUNDAMENTALS_GRID_DETECTOR_H