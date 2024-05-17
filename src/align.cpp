#include "ros/ros.h"
#include <cstdlib>
#include <signal.h>
#include <deque>
#include "sensor_msgs/LaserScan.h"
#include "create_fundamentals/DiffDrive.h"
#include "grid_detector/grid_detector.h"
#include "classes/vector.h"
#include "classes/driver.h"


const float wheel_radius = 0.0308;
const float wheel_base = 0.265;
const float revolution_dist = wheel_radius * M_PI * 2;

void stop_driving(int sig) {
    ROS_DEBUG("stop_driving");
    ros::NodeHandle n;
    ros::ServiceClient diffDrive = n.serviceClient<create_fundamentals::DiffDrive>("diff_drive");

    create_fundamentals::DiffDrive srv;
    srv.request.left = 0;
    srv.request.right = 0;

    diffDrive.call(srv);
    ros::shutdown();
    exit(1);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "align");
    ros::NodeHandle n;

    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
        ros::console::notifyLoggerLevelsChanged();
    }

    ROS_DEBUG("Register grid_detector as subscriber"); 
    GridDetector grid_detector;
    ros::Subscriber sub = n.subscribe("scan_filtered", 1, &GridDetector::get_measurements, &grid_detector);

    ROS_DEBUG("Register Driver as subscriber"); 
    Driver driver(n);
    ros::Subscriber sensorSub = n.subscribe("sensor_packet", 1, &Driver::calculate_wheel_speeds, &driver);

    signal(SIGINT, stop_driving);

    ros::spinOnce();
    ROS_DEBUG("Entering while loop to wait for midpoint."); 

    std::list<GridDetectorResponse> successful_responses;

    int measurement_amount = 10;

    while (successful_responses.size() < measurement_amount) {
        GridDetectorResponse response = grid_detector.detect();

        if (response.success) {
            successful_responses.push_back(response);
        }

        ros::spinOnce();
    }

    Eigen::Vector2f offset {0.0, 0.0};
    float theta = 0.0;

    while (successful_responses.size() > 0) {
        GridDetectorResponse response = successful_responses.front();
        successful_responses.pop_front();

        offset = offset + response.offset / measurement_amount;
        theta = theta + response.theta / measurement_amount;
    }

    ROS_INFO("midpoint: offset = (%f, %f), theta = %f", offset[0], offset[1], theta * 180 / M_PI);

    ROS_DEBUG("drive_to_relative_point"); 
    std::deque<wheelCommand> drive_commands;

    float midpoint_direction_theta = acos(offset[0] / offset.norm());
    
    float pre_turn_distance = wheel_base / 2 * midpoint_direction_theta;    
    float drive_distance = offset.norm();
    float post_turn_distance = wheel_base / 2 * (theta - midpoint_direction_theta);  

    wheelCommand pre_turn_command;
    pre_turn_command.left_wheel = -1 * pre_turn_distance / revolution_dist * 2 * M_PI;
    pre_turn_command.right_wheel = pre_turn_distance / revolution_dist * 2 * M_PI;
    drive_commands.push_back(pre_turn_command);

    wheelCommand drive_command;
    drive_command.left_wheel = drive_distance / revolution_dist * 2 * M_PI;
    drive_command.right_wheel = drive_distance / revolution_dist * 2 * M_PI;
    drive_commands.push_back(drive_command);

    wheelCommand post_turn_command;
    post_turn_command.left_wheel = -1 * post_turn_distance / revolution_dist * 2 * M_PI;
    post_turn_command.right_wheel = post_turn_distance / revolution_dist * 2 * M_PI;
    drive_commands.push_back(post_turn_command);

    while(!drive_commands.empty()) {
        wheelCommand current_command = drive_commands.front();
        drive_commands.pop_front();

        driver.execute_command(current_command);
    }

    return 0;
}
