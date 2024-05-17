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

// void center_aligned_in_cell(Vector dest_point, Vector align_direction) {
//     // drive to destination point
//     const float wheel_radius = 0.0308;
//     const float wheel_base = 0.265;
//     float revolution_dist = wheel_radius * M_PI * 2;

//     Vector vec_x = {1, 0};

//     float theta = std::acos(vec_x.scalar_product(dest_point) / (vec_x.get_length() * dest_point.get_length()));
//     if(dest_point.cross_product(align_direction) < 0) theta = -theta; 
//     float turn_distance = wheel_base / 2 * theta;

//     wheelCommand turn_command;
//     turn_command.left_wheel = -1 * turn_distance / revolution_dist * 2 * M_PI;
//     turn_command.right_wheel = turn_distance / revolution_dist * 2 * M_PI;
//     drive_commands.push_back(turn_command);

//     wheelCommand direction_command;
//     direction_command.left_wheel = dest_point.get_length() / revolution_dist * 2 * M_PI;
//     direction_command.right_wheel = dest_point.get_length() / revolution_dist * 2 * M_PI;
//     drive_commands.push_back(direction_command);

//     // align towards wall
//     float align_angle = std::acos(dest_point.scalar_product(align_direction) / (dest_point.get_length() * align_direction.get_length()));
//     if(dest_point.cross_product(align_direction) < 0) align_angle = -align_angle; 
//     float align_turn_distance = wheel_base / 2 * align_angle;

//     wheelCommand align_turn_command;
//     align_turn_command.left_wheel = -1 * align_turn_distance / revolution_dist * 2 * M_PI;
//     align_turn_command.right_wheel = align_turn_distance / revolution_dist * 2 * M_PI;
//     drive_commands.push_back(align_turn_command);

//     return;
// }

void rotate(float theta) {
    
}

void drive_to_relative_point(Driver driver, Eigen::Vector2f point) {
    ROS_DEBUG("drive_to_relative_point"); 

    std::deque<wheelCommand> drive_commands;

    float midpoint_direction_theta = acos(point[0] / point.norm());
    
    float pre_turn_distance = wheel_base / 2 * midpoint_direction_theta;
    
    wheelCommand pre_turn_command;
    pre_turn_command.left_wheel = -1 * pre_turn_distance / revolution_dist * 2 * M_PI;
    pre_turn_command.right_wheel = pre_turn_distance / revolution_dist * 2 * M_PI;
    drive_commands.push_back(pre_turn_command);

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

    ROS_DEBUG("Register grid_detector as subscriber"); 
    GridDetector grid_detector;
    ros::Subscriber sub = n.subscribe("scan_filtered", 1, &GridDetector::get_measurements, &grid_detector);

    ROS_DEBUG("Register Driver as subscriber"); 
    Driver driver(n);
    ros::Subscriber sensorSub = n.subscribe("sensorPacket", 1, &Driver::calculate_wheel_speeds, &driver);

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

    drive_to_relative_point(driver, offset);

    // if (mid.x == 0. && mid.y == 0.) { 
    //         mid.x = 0.1;
    //         mid.y = 0.1;
    // }

    // ROS_DEBUG("Midpoint at: (%f, %f)", mid.x, mid.y); 
    // ROS_DEBUG("Ori at: (%f, %f)", ori.x, ori.y); 

    // ROS_DEBUG("Start driving to point"); 
    // center_aligned_in_cell(mid, ori);
    // while(!drive_commands.empty()) {
    //     wheelCommand current_command = drive_commands.front();
    //     drive_commands.pop_front();

    //     driver.execute_command(current_command);
    // }

    return 0;
}
