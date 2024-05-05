#include "ros/ros.h"
#include <cstdlib>
#include <signal.h>
#include <deque>
#include "sensor_msgs/LaserScan.h"
#include "create_fundamentals/DiffDrive.h"
#include "classes/line_detector.h"
#include "classes/vector.h"
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

void center_aligned_in_cell(Vector dest_point, Vector align_direction) {
    // drive to destination point
    const float wheel_radius = 0.0308;
    const float wheel_base = 0.265;
    float revolution_dist = wheel_radius * M_PI * 2;

    Vector vec_x = {1, 0};

    float theta = std::acos(vec_x.scalar_product(dest_point) / (vec_x.get_length() * dest_point.get_length()));
    float theta_in_rad = theta * M_PI / 180;
    if(dest_point.cross_product(align_direction) < 0) theta_in_rad = -theta_in_rad; 
    float turn_distance = wheel_base / 2 * theta_in_rad;

    wheelCommand turn_command;
    turn_command.left_wheel = -1 * turn_distance / revolution_dist * 2 * M_PI;
    turn_command.right_wheel = turn_distance / revolution_dist * 2 * M_PI;
    drive_commands.push_back(turn_command);

    wheelCommand direction_command;
    direction_command.left_wheel = dest_point.get_length() / revolution_dist * 2 * M_PI;
    direction_command.right_wheel = dest_point.get_length() / revolution_dist * 2 * M_PI;
    drive_commands.push_back(direction_command);

    // align towards wall
    float align_angle = std::acos(dest_point.scalar_product(align_direction) / (dest_point.get_length() * align_direction.get_length()));
    float align_angle_in_rad = align_angle * M_PI / 180;
    if(dest_point.cross_product(align_direction) < 0) align_angle_in_rad = -align_angle_in_rad; 
    float align_turn_distance = wheel_base / 2 * align_angle_in_rad;

    wheelCommand align_turn_command;
    align_turn_command.left_wheel = -1 * align_turn_distance / revolution_dist * 2 * M_PI;
    align_turn_command.right_wheel = align_turn_distance / revolution_dist * 2 * M_PI;
    drive_commands.push_back(align_turn_command);
}

/*
std::Pair<Vector, Vector> get_midpoint_from_lines(std::vector<Line> lines, Vector sensor_point) 
{
    std::vector<Line> shiftedLines;
    for (const Line& line : foundLines) {
        Line shiftedLine = displaceLine(line, sensor);
        shiftedLines.push_back(shiftedLine);
    }

    std::vector<std::pair<Line, Line>> linePairs;
    for (size_t i = 0; i < shiftedLines.size(); ++i) {
        for (size_t j = i + 1; j < shiftedLines.size(); ++j) {
            double angleDegrees = angleBetweenVectors(shiftedLines[i].ori, shiftedLines[j].ori);
            if (std::abs(angleDegrees - 90) < thresholdDegrees) {
                linePairs.emplace_back(shiftedLines[i], shiftedLines[j]);
            }
        }
    }

    std::vector<Vector> intersections;
    for (const auto& pair : linePairs) {
        Vector intersection = findLineIntersection(pair.first, pair.second);
        if (intersection != nullptr) {
            intersections.push_back(intersection);
        }
        
    }

    Vector averagePoint = computeMean(intersections);
}
*/

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
        ros::spinOnce();

        while(!drive_commands.empty()) {
            wheelCommand current_command = drive_commands.front();
            drive_commands.pop_front();

            driver.execute_command(current_command);
        }

        signal(SIGINT, stop_driving);
    }

    return 0;
}