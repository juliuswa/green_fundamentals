/*
    This Node controls the robot and moves it to the position of the "carrot" (a pose that is published on the carrot topic).

    Input: Pose -> global_position, Pose -> carrot

    Output: Calls diff_drive
*/

#include <mutex>
#include <random>
#include <chrono>
#include <csignal>

#include "ros/ros.h"
#include "std_srvs/Empty.h"
#include "sensor_msgs/LaserScan.h"
#include "Eigen/Dense"

#include "create_fundamentals/SensorPacket.h"
#include "create_fundamentals/DiffDrive.h"
#include "green_fundamentals/DriveTo.h"
#include "green_fundamentals/Obstacle.h"
#include "robot_constants.h"

using std::chrono::high_resolution_clock;
using std::chrono::duration_cast;
using std::chrono::milliseconds;

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
bool is_obstacle_front = false;
bool is_obstacle_left = false;
bool is_obstacle_right = false;
bool is_obstacle_far_front = false;

Position target{0., 0., 0.};
bool should_rotate = false;

float last_left = 0.0;
float last_right = 0.0;
float current_left = 0.0;
float current_right = 0.0;
bool is_first_encoder_measurement = true;

const float max_speed = 15.0;
float cur_max_speed = 15.0;
const float min_speed = 3.0;
const float slow_distance = 0.2;
const float slow_angle = 2.;

ros::ServiceClient diff_drive_service;
create_fundamentals::DiffDrive wheel_commands;

bool is_arrived() 
{
    return std::abs(target.x - my_position.x) < POS_EPSILON && std::abs(target.y - my_position.y) < POS_EPSILON;
}

void wander() 
{
    float speed = cur_max_speed/2;
    
    if (is_obstacle_right)
    {
        // Turn left
        wheel_commands.request.left = -speed;
        wheel_commands.request.right = speed;
    } 
    else if (is_obstacle_left)
    {
        // Turn right
        wheel_commands.request.left = speed;
        wheel_commands.request.right = -speed;
    }
    else if (is_obstacle_front) 
    {
        // Choose at random if right or left
        std::random_device rand;
        std::mt19937 gen(rand());
        std::uniform_int_distribution<std::mt19937::result_type> dist(0, 1);
        int random_value = dist(gen);        
        int random_sign = (random_value == 0) ? -1 : 1;

        wheel_commands.request.left = random_sign * speed;
        wheel_commands.request.right = random_sign * -speed;
    }  
    else 
    {
        wheel_commands.request.left = speed;
        wheel_commands.request.right = speed;
    }
    
    diff_drive_service.call(wheel_commands);
}

void idle() 
{
    wheel_commands.request.left = 0;
    wheel_commands.request.right = 0;
    
    diff_drive_service.call(wheel_commands);
}

Position calculate_target() {

}

void drive_to()
{
    if (is_arrived()) 
    {
        if (should_rotate) 
        {
            float total_angle = fmod(target.theta - my_position.theta, 2 * M_PI);

            float speed = cur_max_speed * (abs(total_angle) / slow_angle); // slows down when closer than "slow_angle"
            speed = std::min(cur_max_speed, speed);
            speed = std::max(cur_max_speed, speed);   

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
                state = State::IDLE;
            }

            diff_drive_service.call(wheel_commands);
            return;
        } 
        else 
        {
            wheel_commands.request.left = 0.;
            wheel_commands.request.right = 0.;
            state = State::IDLE;

            diff_drive_service.call(wheel_commands);
            return;
        }
    }

    Eigen::Vector2f pos_delta {target.x - my_position.x, target.y - my_position.y};
    Eigen::Vector2f robot_direction {cos(my_position.theta), sin(my_position.theta)};     

    float angle = acos((pos_delta[0] * robot_direction[0] + pos_delta[1] * robot_direction[1]) / pos_delta.norm());

    angle = std::min((float)(M_PI / 2.0), angle); // so it's between 0° and 90°

    float cross_product_z = pos_delta[0] * robot_direction[1] - pos_delta[1] * robot_direction[0];    

    int direction = cross_product_z < 0 ? 1 : -1;

    float exponential_factor = 2 * std::exp(-angle) - 1;
    float linear_factor = - 2 * angle / (M_PI / 2) + 1;

    float factor = (linear_factor + exponential_factor) / 2;

    if (is_obstacle_front) { 
        factor = -1;
    }
    
    float speed = cur_max_speed * (pos_delta.norm() / slow_distance); // slows down when closer than "slow_distance"
    speed = std::min(cur_max_speed, speed);
    speed = std::max(min_speed, speed);     

    if(is_obstacle_far_front) 
    {
        ROS_DEBUG("obstacle far front", angle);
        speed /= 2;
    }
    
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

    diff_drive_service.call(wheel_commands);
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
    is_obstacle_front = obst->front;
    is_obstacle_far_front = obst->far_front;
    is_obstacle_right = obst->right;
    is_obstacle_left = obst->left;
}

bool set_drive_to_callback(green_fundamentals::DriveTo::Request  &req, green_fundamentals::DriveTo::Response &res)
{
    my_position.x = req.x_current;
    my_position.y = req.y_current;
    my_position.theta = req.theta_current;
    
    target.x = req.x_target;
    target.y = req.y_target;
    target.theta = req.theta_target;
    
    ROS_INFO("new target: (%f, %f)", req.x_target, req.y_target);
    should_rotate = req.rotate;
    if(req.slow) {
        cur_max_speed = 5;
    } else {
        cur_max_speed = max_speed;
    }
    
    state = State::DRIVE_TO;
    ros::param::set("mover_drive_to_error", false);
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

void shutdown(int signum) 
{   
    wheel_commands.request.left = 0.;
    wheel_commands.request.right = 0.;
    diff_drive_service.call(wheel_commands);
    ros::shutdown();
    exit(0);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot_mover");
    ros::NodeHandle n;
    signal(SIGINT, shutdown);

    ros::param::set("mover_drive_to_error", false);

    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info)) {
        ros::console::notifyLoggerLevelsChanged();
    }

    ROS_INFO("Initializing...");
    diff_drive_service = n.serviceClient<create_fundamentals::DiffDrive>("diff_drive");

    // Subscribers
    ros::Subscriber sensor_sub = n.subscribe("sensor_packet", 1, sensor_callback);
    ros::Subscriber obstacle_sub = n.subscribe("obstacle", 1, obstacle_callback);

    // Services
    ros::ServiceServer drive_to_service = n.advertiseService("mover_set_drive_to", set_drive_to_callback);
    ros::ServiceServer idle_service = n.advertiseService("mover_set_idle", set_idle_callback);
    ros::ServiceServer wander_service = n.advertiseService("mover_set_wander", set_wander_callback);

    ROS_INFO("Ready to drive.");    
    ros::Rate r(30);
    while(ros::ok()) {
        switch (state)
        {
            case State::IDLE:
                // do nothing
                idle();
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