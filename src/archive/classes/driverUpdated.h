#ifndef GREEN_FUNDAMENTALS_DRIVER_H
#define GREEN_FUNDAMENTALS_DRIVER_H

#include "ros/ros.h"
#include <cstdlib>
#include "sensor_msgs/LaserScan.h"
#include <math.h>
#include "create_fundamentals/DiffDrive.h"
#include "create_fundamentals/SensorPacket.h"
#include "green_fundamentals/Position.h"

struct wheelCommand {
    float left_wheel;
    float right_wheel;
};

class Driver {
private:
    green_fundamentals::Position cur_pos;

    ros::ServiceClient diff_drive;

public:
    Driver(ros::NodeHandle& nh);
    void get_cur_position(const create_fundamentals::SensorPacket::ConstPtr& odometry);
    bool arrived(Eigen::Vector2f& dest);
    void go_to_destination(Eigen::Vector2f& dest, float max_speed);

private:
    void drive(wheelCommand& command);
};

#endif //GREEN_FUNDAMENTALS_DRIVER_H
