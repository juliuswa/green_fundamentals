#include "ros/ros.h"
#include <cstdlib>
#include "create_fundamentals/SensorPacket.h"

float x = 0.;
float y = 0.;
float theta = 0.;

void sensor_callback(const create_fundamentals::SensorPacket::ConstPtr& sensor_packet)
{
  
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "odometry");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("sensor_packet", 1, sensor_callback);
  ros::spin();
  return 0;
}
