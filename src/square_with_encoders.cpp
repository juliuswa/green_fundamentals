#include "ros/ros.h"
#include <cstdlib>
#include "sensor_msgs/LaserScan.h"
#include <math.h>
#include "create_fundamentals/DiffDrive.h"
#include "create_fundamentals/ResetEncoders.h"
#include "create_fundamentals/SensorPacket.h"
#include <signal.h>
#include <deque>
#include <ros/console.h>
#include "classes/driver.h"

std::deque<wheelCommand> drive_commands;

void stop_driving(int sig) {
    ROS_DEBUG("stop_driving");
    ros::NodeHandle n;
    ros::ServiceClient diffDrive = n.serviceClient<create_fundamentals::DiffDrive>("diff_drive");

    create_fundamentals::DiffDrive srv;
    srv.request.left = 0;
    srv.request.right = 0;

    diffDrive.call(srv);
    ros::shutdown();
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "square_with_sensors");
    ros::NodeHandle n;

    ros::ServiceClient encoder_client = n.serviceClient<create_fundamentals::ResetEncoders>("reset_encoders");
    Driver driver(n);
    ros::Subscriber sensorSub = n.subscribe("sensor_packet", 1, &Driver::calculate_wheel_speeds, &driver);


    create_fundamentals::ResetEncoders srv;
    encoder_client.call(srv);

    ROS_DEBUG("finished setting up node.");

    const float square_size = 1.0;
    const float wheel_radius = 0.0308;
    const float wheel_base = 0.265;

    float revolution_dist = wheel_radius * M_PI * 2;
    float right_angle_turn_distance = wheel_base * M_PI / 4;
    
    for (int rounds = 0; rounds < 5; rounds++) {
        for (int side = 0; side < 4; side++) {

            wheelCommand straight_command;
            straight_command.left_wheel = square_size / revolution_dist * 2 * M_PI;
            straight_command.right_wheel = square_size / revolution_dist * 2 * M_PI;

            drive_commands.push_back(straight_command);

            wheelCommand turn_command;
            turn_command.left_wheel = right_angle_turn_distance / revolution_dist * 2 * M_PI;
            turn_command.right_wheel = -1 * right_angle_turn_distance / revolution_dist * 2 * M_PI;

            drive_commands.push_back(turn_command);
        }
    }


    while(ros::ok()) {
        while(!drive_commands.empty()) {
            ros::spinOnce();
            wheelCommand current_command = drive_commands.front();
            drive_commands.pop_front();

            driver.execute_command(current_command);
        }

        signal(SIGINT, stop_driving);
    }

    return 0;
}

