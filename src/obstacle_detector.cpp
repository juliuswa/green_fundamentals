#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

#include "robot_constants.h"
#include "green_fundamentals/Obstacle.h"

ros::Publisher obstacle_pub;

const float theta_offset = - 2.37; 
const float x_offset = 0.13;
float robot_side = 0.08;  // determines at what y coordinate we are counting points to being on the side of the robot
float allowed_distance = 0.1;  // how close to the wall we are allowed to go

std::vector<std::pair<float, float>> laser_coords;

void laser_callback(const sensor_msgs::LaserScan::ConstPtr& laser_scan)
{
    laser_coords.clear();
    for(int i = 0; i < laser_scan->ranges.size(); i++) 
    {
        float r = laser_scan->ranges[i];
        
        if (r != r) continue;  //NaN

        float theta = i * laser_scan->angle_increment + theta_offset;

        float x = (r * std::cos(theta)) + x_offset;
        float y = r * std::sin(theta);

        std::pair<float, float> coord{x, y};

        laser_coords.push_back(coord);
    }

    green_fundamentals::Obstacle msg;
    bool left = false;
    bool front = false;
    bool right = false;
    bool far_front = false;

    for(const std::pair<float, float> point : laser_coords) 
    {
        float dist = std::sqrt(pow(point.first, 2) + pow(point.second, 2));
        if(dist < CASING_RADIUS) continue;
    
        if(point.second > robot_side) {  // left
            left = (dist < (CASING_RADIUS + allowed_distance / 2)) ? true : left;
        } else if(point.second < -robot_side) {  // right
            right = (dist < (CASING_RADIUS + allowed_distance / 2)) ? true : right;
        } else {  // front
            front = (dist < (CASING_RADIUS + allowed_distance)) ? true : front;
            far_front = (dist < (CASING_RADIUS + 2 * allowed_distance)) ? true : far_front;
        }
    }

    msg.left = left;
    msg.front = front;
    msg.right = right;
    msg.far_front = far_front;
    obstacle_pub.publish(msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "obstacle_detection");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("scan_filtered", 1, laser_callback);
    obstacle_pub = n.advertise<green_fundamentals::Obstacle>("obstacle", 1);

    ROS_INFO("Ready to obstacle_detection");
    ros::spin();

    return 0;
}