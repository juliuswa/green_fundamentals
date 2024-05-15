#include "ros/ros.h"
#include <boost/bind.hpp>
#include <cmath>

#include "create_fundamentals/SensorPacket.h"
#include "green_fundamentals/Position.h"
#include "robot_constants.h"

#define EPS 0.001

float x = 0.;
float y = 0.;
float theta = 0.;

float v = 0.;
float w = 0.;

float leftEncoderOld = 0.;
float rightEncoderOld = 0.;
float timestepOld = 0.;

bool isFirstData = true;

void sensor_callback(const create_fundamentals::SensorPacket::ConstPtr& sensor_packet, ros::Publisher& odo_pub)
{ 
  if (isFirstData) {
    leftEncoderOld = msg->encoderLeft;
    rightEncoderOld = msg->encoderRight;
    timestepOld = ros::Time::now();
    firstisFirstDataData = false;
  }

  float deltaDist = 0.0f;
  float deltaX = 0.0f;
  float deltaY = 0.0f;
  float deltaTheta = 0.0f;
  float leftWheelDist = 0.0f;
  float rightWheelDist = 0.0f;
  float wheelDistDiff = 0.0f;

  float timestepNow = ros::Time::now();
  float delta_t = timestepNow - timestepOld;
  timestepOld = timestepNow;
  
  // Calculate Distance Wheels traveled
  float leftEncoderNew = msg->encoderLeft; // rad
  float rightEncoderNew = msg->encoderRight; // rad

  float leftWheelDist = (leftEncoderNew - leftEncoderOld) * WHEEL_RADIUS; // m
  float rightWheelDist = (rightEncoderNew - rightEncoderOld) * WHEEL_RADIUS; // m

  deltaDist = (rightWheelDist + leftWheelDist)/2.; // Distance robot traveled
  wheelDistDiff = rightWheelDist - leftWheelDist; 
  deltaTheta = wheelDistDiff / WHEEL_BASE;

  if (fabs(wheelDistDiff) < EPS) {
    deltaX = deltaDist * cos(theta);
    deltaY = deltaDist * sin(theta);
  } else {
    float turnRadius = (WHEEL_BASE / 2.0) * (leftWheelDist + rightWheelDist) / wheelDistDiff;
    deltaX = turnRadius * (sin(theta + deltaTheta) - sin(theta));
    deltaY = -turnRadius * (cos(theta + deltaTheta) - cos(theta));
  }

  if (fabs(delta_t) > EPS) { // hier noch einstellen was genau das ist
    v = deltaDist / delta_t;
    w = deltaTheta / delta_t;
  } else {
    v = 0;
    w = 0;
  }

  x += deltaX;
  y += deltaY;
  theta += deltaTheta;

  green_fundamentals::Position odo_msg;
  odo_msg.x = x;
  odo_msg.y = y;
  odo_msg.theta = theta;
  odo_pub.publish(odo_msg);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "odometry");
  ros::NodeHandle n;
  ros::Publisher odo_pub = n.advertise<green_fundamentals::Position>("odometry", 1);
  ros::Subscriber sub = n.subscribe("sensor_packet", 1, boost::bind(sensor_callback, _1, odo_pub));
  ros::spin();
  return 0;
}
