#include "ros/ros.h"
#include <cstdlib>
#include "sensor_msgs/LaserScan.h"
#include <math.h>
#include "create_fundamentals/DiffDrive.h"
#include "create_fundamentals/SensorPacket.h"

bool obstacle_detected = false;
int obstacle_index = 0;
float obstacle_degree = 0.0;
float obstacle_distance = 10.0;

void drive (ros::ServiceClient& diffDrive, int left, int right) {
    create_fundamentals::DiffDrive srv;
    srv.request.left = left;
    srv.request.right = right;
    diffDrive.call(srv);
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  int closest_index = 0;
  float closest_value = 10.0;
  float threshold = 0.1;

  int center_index = 400;
  float index_range = (M_PI / 2.) / msg->angle_increment;

  for (int i = center_index - index_range; i <= center_index + index_range; i++)
  {
      if (msg->ranges[i] > 0.009 && msg->ranges[i] < closest_value) {
          closest_value = msg->ranges[i];
          // Rechts ist negativ
          // LInks ist positiv
          closest_index = -1 * (i - center_index);
      }
  }

  if (closest_value < threshold) {
    obstacle_detected = true;
    obstacle_degree = closest_index * msg->angle_increment;
    obstacle_index = closest_index;
    obstacle_distance = closest_value;
    ROS_INFO("Obstacle Detected");
  }
  else {
    obstacle_detected = false;
  }

  ROS_INFO("closest index: %d, closest_value: %f", closest_index, closest_value);
  return;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "wanderer");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("scan_filtered", 1, laserCallback);
  ros::ServiceClient diffDrive = n.serviceClient<create_fundamentals::DiffDrive>("diff_drive");

  while (true) {
      if (obstacle_detected) {
        drive(diffDrive, 10, -10);
        ROS_INFO("Spinning");
      }
      else {
        drive(diffDrive, 10, 10);
        ROS_INFO("Waiting");
      }
      ros::spinOnce();
  }
  
  
  return 0;
}
