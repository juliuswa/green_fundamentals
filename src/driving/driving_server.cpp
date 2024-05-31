#include "ros/ros.h"
#include <boost/bind.hpp>
#include <cmath>

#include "../Eigen/Dense"

#include "create_fundamentals/SensorPacket.h"
#include "create_fundamentals/DiffDrive.h"
#include "green_fundamentals/DriveTo.h"
#include "green_fundamentals/Position.h"
#include "../robot_constants.h"

const float max_speed = 10.0;
const float position_precision = 0.03;

bool is_target_set = false;

ros::ServiceClient diff_drive_service;

float x_target = 0.0;
float y_target = 0.0;
float theta_target = 0.0;

bool rotate = false;

struct WheelCommand {
    float left_wheel;
    float right_wheel;
};

void drive(WheelCommand& command) {
    create_fundamentals::DiffDrive srv;
    srv.request.left = command.left_wheel;
    srv.request.right = command.right_wheel;
    diff_drive_service.call(srv);
}

bool arrived(const green_fundamentals::Position::ConstPtr& position) {
    return std::abs(x_target - position->x) < position_precision && std::abs(y_target - position->y) < position_precision;
}

void drive_to_target(const green_fundamentals::Position::ConstPtr& position)
{ 
    if (!is_target_set) return;
    ROS_INFO("current position: x=%f, y=%f, theta=%f deg.", position->x, position->y, position->theta * 180 / M_PI);
    ROS_INFO("current target: x=%f, y=%f, theta=%f", x_target, y_target, theta_target);

    if (!arrived(position)) {
        Eigen::Vector2f pos_delta {x_target - position->x, y_target - position->y};
        ROS_INFO("not arrived. delta: (%f, %f)", pos_delta[0],  pos_delta[1]);

        Eigen::Vector2f robot_direction {cos(position->theta), sin(position->theta)};
        //ROS_INFO("robot_direction: (%f, %f)", robot_direction[0],  robot_direction[1]);       

        float angle = acos((pos_delta[0] * robot_direction[0] + pos_delta[1] * robot_direction[1]) / pos_delta.norm());
        //ROS_INFO("angle = %f deg.", angle * 180 / M_PI);

        angle = std::min((float)(M_PI / 2.0), angle); // so it's between 0° and 90°
        //ROS_INFO("capped angle = %f deg.", angle * 180 / M_PI);  

        float cross_product_z = pos_delta[0] * robot_direction[1] - pos_delta[1] * robot_direction[0];
        //ROS_INFO("cross_product_z = %f * %f - %f * %f = %f", pos_delta[0], robot_direction[1], pos_delta[1], robot_direction[0], cross_product_z);        

        int direction = cross_product_z < 0 ? -1 : 1;
        //ROS_INFO("direction = %d", direction);        

        float factor = 1 - 2 * std::abs(angle) / (M_PI / 2);
        //ROS_INFO("factor = %f", factor);

        if (angle < 0) {
            WheelCommand command = {max_speed, factor * max_speed};
            drive(command);
        }
        else {
            WheelCommand command = {factor * max_speed, max_speed};
            drive(command);
        }
    }
    else if (rotate) {
        ROS_INFO("rotating.");

        float angle = theta_target - position->theta;

        float rotation = fmod(angle + M_PI, 2 * M_PI) - M_PI;

        if (rotation > 0.3) {
            WheelCommand command = {-max_speed, max_speed};
            drive(command);
        }
        else if (rotation < -0.3) {
            WheelCommand command = {max_speed, -max_speed};
            drive(command);
        }
        else {
            WheelCommand command = {0.0, 0.0};
            drive(command);
            ROS_INFO("arrived.");
            is_target_set = false;
        }
    } 
    else {
        ROS_INFO("arrived.");
        WheelCommand command = {0.0, 0.0};
        drive(command);
        is_target_set = false;
    }
}

bool set_target_position(green_fundamentals::DriveTo::Request  &req, green_fundamentals::DriveTo::Response &res) {
    x_target = req.x_target;
    y_target = req.y_target;
    rotate = req.rotate;

    float raw_theta = req.theta_target - floor(req.theta_target / (2 * M_PI)) * (2 * M_PI)

    raw_theta = raw_theta - floor(raw_theta / (2.0 * M_PI)) * 2 * M_PI;  // modulo 2 pi
    //ROS_INFO("mod raw_theta=%f", raw_theta);

    theta_target = raw_theta;
    //ROS_INFO("theta_target=%f", theta_target);

    is_target_set = true;
    return true;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "driving_server");
    ros::NodeHandle n;

    ros::Subscriber odometry_sub = n.subscribe("odometry", 1, drive_to_target);
    ros::ServiceServer drive_to_service = n.advertiseService("drive_to", set_target_position);

    diff_drive_service = n.serviceClient<create_fundamentals::DiffDrive>("diff_drive");

    ros::spin();

    return 0;
}
