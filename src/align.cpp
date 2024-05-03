#include "ros/ros.h"
#include <cstdlib>
#include "sensor_msgs/LaserScan.h"
#include <signal.h>
#include "classes/grid_detector.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "wanderer");
    ros::NodeHandle n;

    GridDetector grid_detector();
    ros::Subscriber sub = n.subscribe("scan_filtered", 1, &GridDetector::detect_walls, &grid_detector);

    while (true) {

        ros::spinOnce();
    }

    return 0;
}