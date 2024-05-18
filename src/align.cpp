#include <cstdlib>
#include <signal.h>
#include <deque>

#include "ros/ros.h"
#include "Eigen/Dense"
#include "sensor_msgs/LaserScan.h"
#include "create_fundamentals/DiffDrive.h"
#include "green_fundamentals/DetectGrid.h"
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

void rotate(float theta, Driver& driver) {
    float turn_distance = wheel_base / 2 * theta;

    wheelCommand turn_command;
    turn_command.left_wheel = -1 * turn_distance / revolution_dist * 2 * M_PI;
    turn_command.right_wheel = turn_distance / revolution_dist * 2 * M_PI;

    driver.execute_command(turn_command);
}

void align_at_position(Eigen::Vector2f position, float theta, Driver& driver) {
    ROS_INFO("midpoint: offset = (%f, %f), theta = %f", position[0], position[1], theta * 180 / M_PI);

    ROS_DEBUG("drive_to_relative_point"); 
    std::deque<wheelCommand> drive_commands;

    float midpoint_direction_theta = acos(position[0] / position.norm());
    
    float pre_turn_distance = wheel_base / 2 * midpoint_direction_theta;    
    float drive_distance = position.norm();
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
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "align");
    ros::NodeHandle n;

    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
        ros::console::notifyLoggerLevelsChanged();
    }

    // ROS_DEBUG("Register grid_detector as subscriber"); 
    // GridDetector grid_detector;
    // ros::Subscriber sub = n.subscribe("scan_filtered", 1, &GridDetector::get_measurements, &grid_detector);

    ROS_DEBUG("Register Driver as subscriber"); 
    Driver driver(n);
    ros::Subscriber sensorSub = n.subscribe("sensor_packet", 1, &Driver::calculate_wheel_speeds, &driver);

    ros::ServiceClient grid_detection_service = n.serviceClient<green_fundamentals::DetectGrid>("detect_grid");

    signal(SIGINT, stop_driving);

    int retry_count = 0;

    while (retry_count < 4) {

        green_fundamentals::DetectGrid srv;

        if (!grid_detection_service.call(srv))
        {
            ROS_ERROR("detecting grid failed.");

            Eigen::Vector2f midpoint {0.0, 0.0};
            rotate(M_PI / 2, driver);

            retry_count += 1;
            continue;
        }

        ROS_INFO("grid position: x_offset=%f, y_offset=%f, theta_offset=%f deg.", 
            srv.response.x_offset, 
            srv.response.y_offset, 
            srv.response.theta_offset * 180 / M_PI);

        Eigen::Vector2f midpoint {srv.response.x_offset, srv.response.y_offset};
        align_at_position(midpoint, srv.response.theta_offset, driver);
        
        break;
    }

    return 0;
}
