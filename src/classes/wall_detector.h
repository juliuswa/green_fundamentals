#ifndef GREEN_FUNDAMENTALS_WALL_DETECTOR_H
#define GREEN_FUNDAMENTALS_WALL_DETECTOR_H

#include "ros/ros.h"
#include <cstdlib>
#include "sensor_msgs/LaserScan.h"
#include <math.h>

class WallDetector {
private:

public:
    WallDetector(ros::NodeHandle& nh);
    void detect_walls(const sensor_msgs::LaserScan::ConstPtr& laser_scan);

private:
};

#endif //GREEN_FUNDAMENTALS_WALL_DETECTOR_H