#include "ros/ros.h"
#include <cstdlib>
#include "create_fundamentals/DiffDrive.h"
#include "create_fundamentals/SensorPacket.h"

void sensorCallback(const create_fundamentals::SensorPacket::ConstPtr& msg)
{
  ROS_INFO("left encoder: %f, right encoder: %f", msg->encoderLeft, msg->encoderRight);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "example");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("sensor_packet", 1, sensorCallback);
  ros::ServiceClient diffDrive = n.serviceClient<create_fundamentals::DiffDrive>("diff_drive");
  
  create_fundamentals::DiffDrive srv;

  ROS_INFO("diffDrive 10 10");
  srv.request.left = 10;
  srv.request.right = 10;
  diffDrive.call(srv);

  ros::Duration(2.0).sleep();
  
  ROS_INFO("diffDrive 0 0");
  srv.request.left = 0;
  srv.request.right = 0;
  diffDrive.call(srv);

  ros::spin();
  return 0;
}
