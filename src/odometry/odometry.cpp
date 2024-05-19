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
ros::Time last_timestamp;

bool isFirstData = true;

void sensor_callback(const create_fundamentals::SensorPacket::ConstPtr& sensor_packet)
{ 
  if (isFirstData) {
    last_left = sensor_packet->encoderLeft;
    last_right = sensor_packet->encoderRight;
    last_timestamp = ros::Time::now();
    isFirstData = false;
  }

  ros::Time new_timestamp = ros::Time::now();

  ros::Duration delta_time = new_timestamp - last_timestamp;
  last_timestamp = new_timestamp;
  
  // Calculate Distance Wheels traveled
  float new_left = sensor_packet->encoderLeft; // rad
  float new_right = sensor_packet->encoderRight; // rad

  float distance_left = (new_left - last_left) * WHEEL_RADIUS; // m
  float distance_right = (new_right - last_right) * WHEEL_RADIUS; // m
  float distance_center = (distance_left + distance_right) / 2;

  float delta_left_right =  distance_right - distance_left; 
  float delta_theta = delta_left_right / WHEEL_BASE;

  float delta_x = 0.0;
  float delta_y = 0.0;
  
  if (fabs(delta_left_right) < EPS) {
    delta_x = distance_center * cos(theta);
    delta_y = distance_center * sin(theta);
  } 
  else {
    float turn_radius = (WHEEL_BASE / 2.0) * (distance_left + distance_right) / delta_left_right;

    delta_x = turn_radius * (sin(theta + delta_theta) - sin(theta));
    delta_y = -turn_radius * (cos(theta + delta_theta) - cos(theta));
  }

  ROS_DEBUG("delta_x = %f, delta_y = %f, delta_theta = %f", delta_x, delta_y, delta_theta);

  x += delta_x;
  y += delta_y;
  theta = (theta + delta_theta);
  theta = theta - std::floor(theta / (2 * M_PI)) * (2 * M_PI);

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

  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info)) {
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info)) {
    ros::console::notifyLoggerLevelsChanged();
  }

  odometry_pub = n.advertise<green_fundamentals::Position>("odometry", 1);
  ros::Subscriber sub = n.subscribe("sensor_packet", 1, sensor_callback);

  ros::spin();
  return 0;
}
