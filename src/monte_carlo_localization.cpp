#include "ros/ros.h"
#include <boost/bind.hpp>
#include <cmath>
#include "Eigen/Dense"

#include "nav_msgs/OccupancyGrid.h"

ros::Subscriber map_sub;
bool map_received = false;

std::vector<std::vector<int8_t>> map_data;
int map_height, map_width;



void map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    
    map_height = msg->info.height;
    map_width = msg->info.width;

    // Resize the map_data to match the map dimensions
    map_data.resize(map_height);
    for (int i = 0; i < map_height; ++i) {
        map_data[i].resize(map_width);
    }

    // Fill the 2D array with the occupancy data
    for (int y = 0; y < map_height; ++y) {
        for (int x = 0; x < map_width; ++x) {
            // Calculate the index in the 1D data array
            int index = x + y * map_width;
            map_data[y][x] = msg->data[index];
        }
    }

    ROS_INFO("Map received");
    map_received = true;
    map_sub.shutdown();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mc_localization");
    ros::NodeHandle n;

    //ros::Publisher map_pub = n.advertise<nav_msgs::OccupancyGrid>("map", 1);
    map_sub = n.subscribe("grid_map", 1, map_callback);

    ROS_INFO("Starting Node");

    ros::Rate loop_rate(1);  // Hz
    while (ros::ok())
    {
        if (map_received) 
        {
            
        }
        
        ros::spinOnce();
        loop_rate.sleep();
    }
  
    return 0;
}