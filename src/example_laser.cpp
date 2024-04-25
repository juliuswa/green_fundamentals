#include "ros/ros.h"
#include <cstdlib>
#include "sensor_msgs/LaserScan.h"


void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  ROS_INFO("%f", msg->ranges[msg->ranges.size()/2]);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "example");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("scan_filtered", 1, laserCallback);
  ros::spin();
  return 0;
}
