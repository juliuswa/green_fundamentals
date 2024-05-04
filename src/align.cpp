#include "ros/ros.h"
#include <cstdlib>
#include "sensor_msgs/LaserScan.h"
#include "create_fundamentals/DiffDrive.h"
#include <signal.h>
#include "classes/line_detector.h"
#include "classes/vector.h"
#include "classes/driver.h"
#include <deque>

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

void drive_align_at_point(Vector dest_point, Vector align_direction) {
    // drive to destination point
    const float wheel_radius = 0.0308;
    const float wheel_base = 0.265;
    float revolution_dist = wheel_radius * M_PI * 2;  // Umfang rad

    Vector vec_x = {1, 0};

    float theta = std::acos(vec_x.scalar_product(dest_point) / (vec_x.get_length() * dest_point.get_length()));
    float theta_in_rad = theta * M_PI / 180;
    float turn_distance = wheel_base / 2 * theta_in_rad;

    wheelCommand turn_command;
    turn_command.left_wheel = turn_distance / revolution_dist * 2 * M_PI;
    turn_command.right_wheel = -1 * turn_distance / revolution_dist * 2 * M_PI;
    drive_commands.push_back(turn_command);

    wheelCommand direction_command;
    direction_command.left_wheel = dest_point.get_length() / revolution_dist * 2 * M_PI;
    direction_command.right_wheel = dest_point.get_length() / revolution_dist * 2 * M_PI;
    drive_commands.push_back(direction_command);

    // align towards wall
    float align_angle = std::acos(dest_point.scalar_product(align_direction) / (dest_point.get_length() * align_direction.get_length()));
    float align_angle_in_rad = align_angle * M_PI / 180;
    float align_turn_distance = wheel_base / 2 * align_angle_in_rad;

    wheelCommand align_turn_command;
    align_turn_command.left_wheel = align_turn_distance / revolution_dist * 2 * M_PI;
    align_turn_command.right_wheel = -1 * align_turn_distance / revolution_dist * 2 * M_PI;
    drive_commands.push_back(align_turn_command);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "wanderer");
    ros::NodeHandle n;

    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
        ros::console::notifyLoggerLevelsChanged();
    }

    LineDetector line_detector;
    ros::Subscriber sub = n.subscribe("scan_filtered", 1, &LineDetector::detect, &line_detector);

    Driver driver(n);
    ros::Subscriber sensorSub = n.subscribe("sensorPacket", 1, &Driver::calculate_wheel_speeds, &driver);

    ROS_INFO("subscribed to scan_filtered.");

    while (ros::ok()) {
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