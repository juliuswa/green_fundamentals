#include <cstdlib>
#include <list>

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "green_fundamentals/Vector2f.h"
#include "green_fundamentals/LaserCoordinates.h"

ros::Publisher coordinates_pub;

const float theta_offset = - 2.37; 
const float x_offset = 0.13;

void laser_coordinates(const sensor_msgs::LaserScan::ConstPtr& laser_scan) 
{
    ROS_DEBUG("Received LaserScan");   
    green_fundamentals::LaserCoordinates coor_msg;
    
    for(int i = 0; i < laser_scan->ranges.size(); i++) {
        float r = laser_scan->ranges[i];
        
        if (r != r) continue;  //NaN

        float theta = i * laser_scan->angle_increment + theta_offset;

        green_fundamentals::Vector2f vector_msg;
        vector_msg.x = (r * std::cos(theta)) + x_offset;
        vector_msg.y = r * std::sin(theta);

        coor_msg.coordinates.push_back(vector_msg);
    }

    ROS_DEBUG("%ld measurements taken.", coor_msg.coordinates.size()); 
    coordinates_pub.publish(coor_msg);

    return;
}

int main(int argc, char **argv)
{  
    ros::init(argc, argv, "calculate_laser_coordinates");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("scan_filtered", 1, laser_coordinates);
    coordinates_pub = n.advertise<green_fundamentals::LaserCoordinates>("laser_coordinates", 1);

    ROS_INFO("Ready to calculate_laser_coordinates");
    ros::spin();

    return 0;
}