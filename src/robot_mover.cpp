/*
    This Node controls the robot and moves it to the position of the "carrot" (a pose that is published on the carrot topic).

    Input: Pose -> global_position, Pose -> carrot

    Output: Calls diff_drive
*/

#include <mutex>

#include "ros/ros.h"
#include "std_srvs/Empty.h"
#include "sensor_msgs/LaserScan.h"
#include "Eigen/Dense"

#include "create_fundamentals/SensorPacket.h"
#include "create_fundamentals/DiffDrive.h"
#include "green_fundamentals/DriveTo.h"
#include "green_fundamentals/Obstacle.h"
#include "robot_constants.h"

struct Position {
    float x, y, theta;
};

enum State {
    IDLE,
    DRIVE_TO,
    WANDER
};

State state = State::IDLE;

Position my_position{0., 0., 0.};
bool is_obstacle_in_front = false;
Position target{0., 0., 0.};
bool should_rotate = false;

float last_left = 0.0;
float last_right = 0.0;
float current_left = 0.0;
float current_right = 0.0;
bool is_first_encoder_measurement = true;

const float max_speed = 14.0;
const float min_speed = 3.0;
const float slow_distance = 0.2;
const float slow_angle = 1.5;

float min_distance = -1;
float min_distance_angle = 0;
const float distance_threshold = 0.1;

ros::ServiceClient diff_drive_service;
create_fundamentals::DiffDrive wheel_commands;

bool is_arrived() 
{
    return std::abs(target.x - my_position.x) < POS_EPSILON && std::abs(target.y - my_position.y) < POS_EPSILON;
}

void wander() 
{
    float speed = max_speed/2;
    if (min_distance < distance_threshold)
    {
        if (min_distance_angle < 0.)
        {
            // Turn left
            wheel_commands.request.left = -speed;
            wheel_commands.request.right = speed;
        } 
        else
        {
            // Turn right
            wheel_commands.request.left = speed;
            wheel_commands.request.right = -speed;
        }
        
    }
    else 
    {
        wheel_commands.request.left = speed;
        wheel_commands.request.right = speed;
    }
    
    diff_drive_service.call(wheel_commands);
}

void drive_to()
{
    if (!is_arrived()) 
    {
        if (is_obstacle_in_front) {
            state = State::IDLE;
            return;
        }

        Eigen::Vector2f pos_delta {target.x - my_position.x, target.y - my_position.y};
        Eigen::Vector2f robot_direction {cos(my_position.theta), sin(my_position.theta)};     

        float angle = acos((pos_delta[0] * robot_direction[0] + pos_delta[1] * robot_direction[1]) / pos_delta.norm());

        angle = std::min((float)(M_PI / 2.0), angle); // so it's between 0° and 90°

        float cross_product_z = pos_delta[0] * robot_direction[1] - pos_delta[1] * robot_direction[0];    

        int direction = cross_product_z < 0 ? 1 : -1;

        float exponential_factor = 2 * std::exp(-angle) - 1;
        float linear_factor = - 2 * angle / (M_PI / 2) + 1;

        float factor = (exponential_factor + linear_factor) / 2;
        
        float speed = max_speed * (pos_delta.norm() / slow_distance); // slows down when closer than "slow_distance"
        speed = std::min(max_speed, speed);
        speed = std::max(min_speed, speed);        
        
        if (direction < 0) 
        {
            wheel_commands.request.left = speed;
            wheel_commands.request.right = factor * speed;
        }
        else 
        {
            wheel_commands.request.left = factor * speed;
            wheel_commands.request.right = speed;
        }
    }
    else if (should_rotate) 
    {
        float total_angle = target.theta - my_position.theta;

        float speed = max_speed * (abs(total_angle) / slow_angle); // slows down when closer than "slow_angle"
        speed = std::min(max_speed, speed);
        speed = std::max(min_speed, speed);   

        if (total_angle > THETA_EPSILON) {
            wheel_commands.request.left = -speed;
            wheel_commands.request.right = speed;
        }
        else if (total_angle < -THETA_EPSILON) {
            wheel_commands.request.left = speed;
            wheel_commands.request.right = -speed;
        }
        else {
            wheel_commands.request.left = 0.;
            wheel_commands.request.right = 0.;
        }
    } 
    else 
    {
        wheel_commands.request.left = 0.;
        wheel_commands.request.right = 0.;
        state = State::IDLE;
    }
    diff_drive_service.call(wheel_commands);
}

void laser_callback(const sensor_msgs::LaserScan::ConstPtr& laser_scan)
{   
    float min_distance_temp = 1.;
    float min_angle_temp = 0.;
    for(int i = 0; i < laser_scan->ranges.size(); i++) {
        float distance = laser_scan->ranges[i];
        
        if (distance != distance || distance < 0.03) continue;  //NaN or too close

        float angle = i * laser_scan->angle_increment + laser_scan->angle_min;

        if (distance < min_distance_temp) 
        {
            min_distance_temp = distance;
            min_angle_temp = angle;
        }
    }

    min_distance = min_distance_temp;
    min_distance_angle = min_angle_temp;
}

void sensor_callback(const create_fundamentals::SensorPacket::ConstPtr& msg) 
{
    last_left = current_left;
    last_right = current_right;
    current_left = msg->encoderLeft;
    current_right = msg->encoderRight;
    if (is_first_encoder_measurement) {
        last_left = current_left;
        last_right = current_right;

        is_first_encoder_measurement = false;
        return;
    }

    float distance_left = (current_left - last_left) * WHEEL_RADIUS;
    float distance_right = (current_right - last_right) * WHEEL_RADIUS; 
    float distance = (distance_left + distance_right) / 2;
    float delta_theta = (distance_right - distance_left) / WHEEL_BASE;

    my_position.x += distance * cos(my_position.theta + delta_theta/2);
    my_position.y += distance * sin(my_position.theta + delta_theta/2);
    my_position.theta += delta_theta;
}

void obstacle_callback(const green_fundamentals::Obstacle::ConstPtr& obst) 
{
    
    is_obstacle_in_front = obst->front;

}

bool set_drive_to_callback(green_fundamentals::DriveTo::Request  &req, green_fundamentals::DriveTo::Response &res)
{
    my_position.x = req.x_current;
    my_position.y = req.y_current;
    my_position.theta = req.theta_current;

    target.x = req.x_target;
    target.y = req.y_target;
    target.theta = req.theta_target;
    
    should_rotate = req.rotate;
    
    state = State::DRIVE_TO;
    return true;
}

bool set_idle_callback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    state = State::IDLE;
    return true;
}

bool set_wander_callback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    state = State::WANDER;
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot_mover");
    ros::NodeHandle n;

    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
        ros::console::notifyLoggerLevelsChanged();
    }

    ROS_INFO("Initializing...");
    diff_drive_service = n.serviceClient<create_fundamentals::DiffDrive>("diff_drive");

    // Subscribers
    ros::Subscriber sensor_sub = n.subscribe("sensor_packet", 1, sensor_callback);
    ros::Subscriber scan_sub = n.subscribe("scan_filtered", 1, laser_callback);

    // Services
    ros::ServiceServer drive_to_service = n.advertiseService("mover_set_drive_to", set_drive_to_callback);
    ros::ServiceServer idle_service = n.advertiseService("mover_set_idle", set_idle_callback);
    ros::ServiceServer wander_service = n.advertiseService("mover_set_wander", set_wander_callback);

    ROS_INFO("Ready to drive.");    
    ros::Rate r(15);
    while(ros::ok()) {

        switch (state)
        {
        case State::IDLE:
            // do nothing
            break;

        case State::DRIVE_TO:
            drive_to();
            break;

        case State::WANDER:
            wander();
            break;
        }

        ros::spinOnce();
        r.sleep();
    }

    return 0;
}