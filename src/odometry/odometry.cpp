#include "ros/ros.h"
#include <boost/bind.hpp>
#include <cmath>

#include "create_fundamentals/SensorPacket.h"
#include "green_fundamentals/Position.h"
#include "../robot_constants.h"

#define EPS 0.001

ros::Publisher odometry_pub;

float x = 0.;
float y = 0.;
float theta = 0.;

float last_left;
float last_right;

bool isFirstData = true;

void sensor_callback(const create_fundamentals::SensorPacket::ConstPtr& sensor_packet)
{ 
  if (isFirstData) {
    last_left = sensor_packet->encoderLeft;
    last_right = sensor_packet->encoderRight;
    isFirstData = false;
  }
  
  // Calculate Distance Wheels traveled
  float new_left = sensor_packet->encoderLeft; // rad
  float new_right = sensor_packet->encoderRight; // rad

  float distance_left = (new_left - last_left) * WHEEL_RADIUS; // m
  float distance_right = (new_right - last_right) * WHEEL_RADIUS; // m
  float distance = (distance_left + distance_right) / 2;

  float delta_theta = (distance_right - distance_left) / WHEEL_BASE;

  x += distance * cos(theta + delta_theta/2);
  y += distance * sin(theta + delta_theta/2);
  theta += delta_theta;
  ROS_INFO("\nx     = %f\ny     = %f\ntheta = %f", x, y, theta);

  last_left = new_left;
  last_right = new_right;

  green_fundamentals::Position odo_msg;
  odo_msg.x = x;
  odo_msg.y = y;
  odo_msg.theta = theta;
  odometry_pub.publish(odo_msg);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "odometry");
  ros::NodeHandle n;

  odometry_pub = n.advertise<green_fundamentals::Position>("odometry", 1);
  ros::Subscriber sub = n.subscribe("sensor_packet", 1, sensor_callback);

  ros::spin();
  return 0;
}
