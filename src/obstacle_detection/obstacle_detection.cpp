#include <cstdlib>
#include <list>

#include "ros/ros.h"
#include "green_fundamentals/Vector2f.h"
#include "green_fundamentals/LaserCoordinates.h"
#include "green_fundamentals/Obstacle.h"

ros::Publisher obstacle_pub;
float robot_radius = 0.335 / 2;
float allowed_distance = 0.06;

void detect_obstacle(const green_fundamentals::LaserCoordinates::ConstPtr& coordinates) { 
    green_fundamentals::Obstacle msg;
    msg.left = false;
    msg.front = false;
    msg.right = false;

    for(green_fundamentals::Vector2f point : coordinates->coordinates) {
        float dist = std::sqrt(pow(point.x, 2) + pow(point.y, 2));

        if(point.y > robot_radius) {
            // left
            msg.left = dist < (robot_radius + allowed_distance / 2) ? true : msg.left;
        } else if(point.y < -robot_radius) {
            // right
            msg.right = dist < (robot_radius + allowed_distance / 2) ? true : msg.right;
        } else {  
            // front
            msg.front = dist < (robot_radius + allowed_distance) ? true : msg.front;
        }
        ROS_DEBUG("dist: %f, x: %f, y: %f", dist, point.x, point.y);
        ROS_DEBUG("left: %d, front: %d, rigt: %d", msg.left, msg.front, msg.right);
    }
    
    obstacle_pub.publish(msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "obstacle_detection");
    ros::NodeHandle n;

    ros::Subscriber coordinates_sub = n.subscribe("laser_coordinates", 1, detect_obstacle);
    obstacle_pub = n.advertise<green_fundamentals::LaserCoordinates>("obstacle", 1);

    ros::spin();

    return 0;
}
