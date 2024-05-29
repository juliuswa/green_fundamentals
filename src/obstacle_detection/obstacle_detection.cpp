#include <cstdlib>
#include <list>

#include "ros/ros.h"
#include "../robot_constants.h"
#include "green_fundamentals/Vector2f.h"
#include "green_fundamentals/LaserCoordinates.h"
#include "green_fundamentals/Obstacle.h"

ros::Publisher obstacle_pub;
float robot_side = 0.08;  // determines at what y coordinate we are counting points to being on the side of the robot
float allowed_distance = 0.1;  // how close to the wall we are allowed to go

void detect_obstacle(const green_fundamentals::LaserCoordinates::ConstPtr& coordinates) { 
    green_fundamentals::Obstacle msg;
    bool left = false;
    bool front = false;
    bool right = false;

    for(const green_fundamentals::Vector2f point : coordinates->coordinates) {
        float dist = std::sqrt(pow(point.x, 2) + pow(point.y, 2));
        if(dist < CASING_RADIUS) continue;
    
        if(point.y > robot_side) {
            left = (dist < (CASING_RADIUS + allowed_distance / 2)) ? true : left;
        } else if(point.y < -robot_side) {
            right = (dist < (CASING_RADIUS + allowed_distance / 2)) ? true : right;
        } else {
            front = (dist < (CASING_RADIUS + allowed_distance)) ? true : front;
        }
    }

    msg.left = left;
    msg.front = front;
    msg.right = right;
    ROS_INFO("-------------------------------------");
    ROS_INFO("left: %d, front: %d, rigt: %d", msg.left, msg.front, msg.right);
    obstacle_pub.publish(msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "obstacle_detection");
    ros::NodeHandle n;

    ros::Subscriber coordinates_sub = n.subscribe("laser_coordinates", 1, detect_obstacle);
    obstacle_pub = n.advertise<green_fundamentals::Obstacle>("obstacle", 1);

    ros::spin();

    return 0;
}
