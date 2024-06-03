#include "ros/ros.h"
#include <boost/bind.hpp>
#include <cmath>

#include "../Eigen/Dense"

#include "create_fundamentals/SensorPacket.h"
#include "create_fundamentals/DiffDrive.h"
#include "green_fundamentals/DriveTo.h"
#include "green_fundamentals/Position.h"
#include "../robot_constants.h"

const float max_speed = 14.0;
const float min_speed = 3.0;

const float slow_distance = 0.2;
const float slow_angle = 1.5;

const float position_precision = 0.03;
const float theta_precision = 0.03;

ros::ServiceClient diff_drive_service;

float last_left_encoder = 0.0;
float last_right_encoder = 0.0;

float x_current = 0.0;
float y_current = 0.0;
float theta_current = 0.0;

bool has_target = false;

float x_target = 0.0;
float y_target = 0.0;
float theta_target = 0.0;

bool rotate = false;

struct WheelCommand {
    float left_wheel;
    float right_wheel;
};

void set_odometry(const create_fundamentals::SensorPacket::ConstPtr& sensor_packet) {
    ROS_DEBUG("set_odometry");
    float new_left = sensor_packet->encoderLeft; // rad
    float new_right = sensor_packet->encoderRight; // rad

    float distance_left = (new_left - last_left_encoder) * WHEEL_RADIUS; // m
    float distance_right = (new_right - last_right_encoder) * WHEEL_RADIUS; // m
    float distance = (distance_left + distance_right) / 2;

    float delta_theta = (distance_right - distance_left) / WHEEL_BASE;

    x_current += distance * cos(theta_current + delta_theta/2);
    y_current += distance * sin(theta_current + delta_theta/2);
    theta_current += delta_theta;

    last_left_encoder = new_left;
    last_right_encoder = new_right;
}

bool set_target_position(green_fundamentals::DriveTo::Request  &req, green_fundamentals::DriveTo::Response &res) {
    ROS_DEBUG("set_target_position");
    x_current = 0.0;
    y_current = 0.0;
    theta_current = 0.0;

    x_target = req.x_target;
    y_target = req.y_target;
    theta_target = req.theta_target;
    
    rotate = req.rotate;
    has_target = true;

    return true;
}

void drive(WheelCommand& command) {
    create_fundamentals::DiffDrive srv;
    srv.request.left = command.left_wheel;
    srv.request.right = command.right_wheel;
    diff_drive_service.call(srv);
}

bool arrived() {
    return std::abs(x_target - x_current) < position_precision && std::abs(y_target - y_current) < position_precision;
}

void drive_to_target()
{ 
    if (!has_target) return;
    ROS_INFO("current: x=%f, y=%f, theta=%f", x_current, y_current, theta_current);
    ROS_INFO("target:  x=%f, y=%f, theta=%f", x_target, y_target, theta_target);

    if (!arrived()) {
        Eigen::Vector2f pos_delta {x_target - x_current, y_target - y_current};
        ROS_DEBUG("not arrived. delta: (%f, %f)", pos_delta[0],  pos_delta[1]);

        Eigen::Vector2f robot_direction {cos(theta_current), sin(theta_current)};
        ROS_DEBUG("robot_direction: (%f, %f)", robot_direction[0],  robot_direction[1]);       

        float angle = acos((pos_delta[0] * robot_direction[0] + pos_delta[1] * robot_direction[1]) / pos_delta.norm());
        ROS_DEBUG("angle = %f", angle);

        angle = std::min((float)(M_PI / 2.0), angle); // so it's between 0° and 90°
        ROS_DEBUG("capped angle = %f deg.", angle * 180 / M_PI);  

        float cross_product_z = pos_delta[0] * robot_direction[1] - pos_delta[1] * robot_direction[0];
        ROS_DEBUG("cross_product_z = %f * %f - %f * %f = %f", pos_delta[0], robot_direction[1], pos_delta[1], robot_direction[0], cross_product_z);        

        int direction = cross_product_z < 0 ? 1 : -1;
        ROS_DEBUG("direction = %d", direction);        

        float exponential_factor = 2 * std::exp(-angle) - 1;
        float linear_factor = - 2 * angle / (M_PI / 2) + 1;
        ROS_DEBUG("ex_factor: %f, lin_factor %f", exponential_factor, linear_factor);

        float factor = (exponential_factor + linear_factor) / 2;
        ROS_DEBUG("factor = %f", factor);
        
        float speed = max_speed * (pos_delta.norm() / slow_distance); // slows down when closer than "slow_distance"
        speed = std::min(max_speed, speed);
        speed = std::max(min_speed, speed);

        if (direction < 0) {
            WheelCommand command = {speed, factor * speed};
            drive(command);
        }
        else {
            WheelCommand command = {factor * speed, speed};
            drive(command);
        }
    }
    else if (rotate) {
        ROS_INFO("rotating.");

        float total_angle = theta_target - theta_current;  
        ROS_DEBUG("total_angle %f", total_angle);    

        float speed = max_speed * (abs(total_angle) / slow_angle); // slows down when closer than "slow_angle"
        speed = std::min(max_speed, speed);
        speed = std::max(min_speed, speed);
        ROS_DEBUG("rotation speed %f", speed);    

        if (total_angle > theta_precision) {
            WheelCommand command = {-speed, speed};
            drive(command);
        }
        else if (total_angle < -theta_precision) {
            WheelCommand command = {speed, -speed};
            drive(command);
        }
        else {
            WheelCommand command = {0.0, 0.0};
            drive(command);

            ROS_INFO("finished rotating.");
            rotate = false;
        }
    } 
    else {
        ROS_INFO("arrived.");
        WheelCommand command = {0.0, 0.0};
        drive(command);
        has_target = false;
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "driving_server");
    ros::NodeHandle n;

    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
        ros::console::notifyLoggerLevelsChanged();
    }

    ros::Subscriber sensor_sub = n.subscribe("sensor_packet", 1, set_odometry);
    ros::ServiceServer drive_to_service = n.advertiseService("drive_to", set_target_position);

    diff_drive_service = n.serviceClient<create_fundamentals::DiffDrive>("diff_drive");

    ROS_INFO("Ready to drive.");
    
    ros::Rate r(30);
    while(ros::ok()) {
        drive_to_target();

        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
