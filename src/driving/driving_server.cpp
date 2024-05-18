#include "ros/ros.h"
#include <boost/bind.hpp>
#include <cmath>

#include "create_fundamentals/SensorPacket.h"
#include "create_fundamentals/DiffDrive.h"
#include "green_fundamentals/DriveTo.h"
#include "green_fundamentals/Position.h"
#include "../robot_constants.h"

const float max_speed = 5.0;

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

void drive_to_target(const green_fundamentals::Position::ConstPtr& position)
{ 
    float angle_to_dest = atan2(y_target - position->x, x_target - position->y); // in rad
    float angle = angle_to_dest - position->theta;
    angle = fmod(angle + M_PI, 2 * M_PI) - M_PI;  // norm it so it's between -180° and 180°

    float percent = angle / M_PI / 4;
    float factor = -1 * (percent - 1);  // this will give 0 for 45°, -1 for 90° and 1 for 0°


    WheelCommand command = {factor * max_speed, max_speed};
    
    if(angle < 0) {  // if right-turn
        WheelCommand command = {max_speed, factor * max_speed};
    } 
    
    drive(command);
}

bool set_target_position(green_fundamentals::DriveTo::Request  &req, green_fundamentals::DriveTo::Response &res) {
    x_target = req.x_target;
    y_target = req.y_target;
    theta_target = req.theta_target;

    return true;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "driving_server");
    ros::NodeHandle n;

    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
        ros::console::notifyLoggerLevelsChanged();
    }

    ros::Subscriber odometry_sub = n.subscribe("odometry", 1, drive_to_target);
    ros::ServiceServer drive_to_service = n.advertiseService("drive_to", set_target_position);

    diff_drive_service = n.serviceClient<create_fundamentals::DiffDrive>("diff_drive");

    while (ros::ok()) {
        ros::spinOnce();
    }

    return 0;
}
