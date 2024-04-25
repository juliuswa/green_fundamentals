#include "ros/ros.h"
#include <cstdlib>
#include "create_fundamentals/DiffDrive.h"
#include "create_fundamentals/SensorPacket.h"

int sees_obstacle(sensor_msgs::LaserScan::ConstPtr& msg)
{   
    int closest_index = 0;
    float closest_value = 10.0;

    for (int i = 44; i < msg->ranges.size(); i++)
    {
       if (msg->ranges[i] < closest_value) {
            closest_value = msg->ranges[i];
            closest_index = i;
       }



    }

    ROS_INFO("closest index: %d, closest_value: %f", closest_index, closest_value);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "wanderer");

  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("scan_filtered", 1, sees_obstacle);

  ros::spin();
  return 0;
}
