#ifndef GREEN_FUNDAMENTALS_DRIVER_H
#define GREEN_FUNDAMENTALS_DRIVER_H

#include "ros/ros.h"
#include <cstdlib>
#include "sensor_msgs/LaserScan.h"
#include <math.h>
#include "create_fundamentals/DiffDrive.h"
#include "create_fundamentals/SensorPacket.h"

struct wheelCommand {
    float left_wheel;
    float right_wheel;
};

class Driver {
private:
    const float spring_constant = 4;
    const float damping_constant = 0.5;
    const float tolerance = 0.2;

    wheelCommand current_command;
    wheelCommand current_did;

    float speed_left = 0.0;
    float speed_right = 0.0;

    float left_encoder_before_command;
    float right_encoder_before_command;

    float left_encoder_current = 0.0;
    float right_encoder_current = 0.0;

    ros::ServiceClient diff_drive;

public:
    Driver(ros::NodeHandle& nh);
    void execute_command(wheelCommand& command);
    void calculate_wheel_speeds(const create_fundamentals::SensorPacket::ConstPtr& sensor_packet);

private:
    void drive();
    bool command_done();
    float calculate_speed(float delta, float velocity);
};

#endif //GREEN_FUNDAMENTALS_DRIVER_H
