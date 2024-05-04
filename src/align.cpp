#include "ros/ros.h"
#include <cstdlib>
#include "sensor_msgs/LaserScan.h"
#include <signal.h>
#include "classes/line_detector.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "wanderer");
    ros::NodeHandle n;

    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
        ros::console::notifyLoggerLevelsChanged();
    }

    LineDetector grid_detector;
    ros::Subscriber sub = n.subscribe("scan_filtered", 1, &LineDetector::detect, &grid_detector);

    ROS_INFO("subscribed to scan_filtered.");

    while (ros::ok()) {
        ros::spinOnce();
    }

    return 0;
}