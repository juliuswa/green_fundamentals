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


float spring_constant = 4;
float damping_constant = 0.5;
float tolerance = 0.2;

float speed_left = 0.0;
float speed_right = 0.0;

float left_encoder_before_command;
float right_encoder_before_command;

float left_encoder_current = 0.0;
float right_encoder_current = 0.0;

std::deque<std::vector<float>> drive_commands; 
std::vector<float> current_command;
std::vector<float> current_did;

void drive(ros::ServiceClient& diffDrive, int left, int right) {
    create_fundamentals::DiffDrive srv;
    srv.request.left = left;
    srv.request.right = right;
    diffDrive.call(srv);
}

float calculate_speed(float delta, float velocity) {
    float p_control = spring_constant * delta;
    float pd_control = p_control - damping_constant * velocity;
    return std::round(pd_control);
}

void calculate_wheel_speeds(const create_fundamentals::SensorPacket::ConstPtr& sensor_packet) {
    ROS_DEBUG("calculate_wheel_speeds");
    speed_left = calculate_speed(current_command[0] - current_did[0], speed_left);
    speed_right = calculate_speed(current_command[1] - current_did[1], speed_right);

    left_encoder_current = sensor_packet->encoderLeft;
    right_encoder_current = sensor_packet->encoderRight;

    current_did[0] = left_encoder_current - left_encoder_before_command;
    current_did[1] = right_encoder_current - right_encoder_before_command;
}

void stop_driving(int sig) {
    ROS_DEBUG("stop_driving");
    ros::NodeHandle n;
    ros::ServiceClient diffDrive = n.serviceClient<create_fundamentals::DiffDrive>("diff_drive");
    drive(diffDrive, 0, 0);
    ros::shutdown();
}

bool command_done() {

    return std::abs(current_command[0] - current_did[0]) < tolerance 
        && std::abs(current_command[1] - current_did[1]) < tolerance
        && !(std::abs(speed_left) > 1.0)
        && !(std::abs(speed_right) > 1.0);
}

int main(int argc, char **argv) {
    ROS_DEBUG("started main.");

    ros::init(argc, argv, "square_with_sensors");
    ros::NodeHandle n;

    ros::Subscriber sensorSub = n.subscribe("sensorPacket", 1, calculate_wheel_speeds);
    ros::ServiceClient diff_drive = n.serviceClient<create_fundamentals::DiffDrive>("diff_drive");
    ros::ServiceClient encoder_client = n.serviceClient<create_fundamentals::ResetEncoders>("reset_encoders");

    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
        ros::console::notifyLoggerLevelsChanged();
    }

    ROS_DEBUG("initialized and subscribed services.");

    create_fundamentals::ResetEncoders srv;
    encoder_client.call(srv);

    ROS_DEBUG("reseted encoders.");

    const float square_size = 1.0;
    const float wheel_radius = 0.0308;
    const float wheel_base = 0.265;

    float revolution_dist  = wheel_radius * M_PI * 2;

    float right_angle_turn_distance = wheel_base * M_PI / 4;

    current_command.push_back(0.0);
    current_command.push_back(0.0);

    current_did.push_back(0.0);
    current_did.push_back(0.0);

    ROS_DEBUG("initialized variables.");
    
    for (int rounds = 0; rounds < 5; rounds++) {
        for (int side = 0; side < 4; side++) {

            std::vector<float> straight_command;
            straight_command.push_back(square_size / revolution_dist * 2 * M_PI);
            straight_command.push_back(square_size / revolution_dist * 2 * M_PI);

            drive_commands.push_back(straight_command);

            std::vector<float> turn_command;
            turn_command.push_back(right_angle_turn_distance / revolution_dist * 2 * M_PI);
            turn_command.push_back(-1 * right_angle_turn_distance / revolution_dist * 2 * M_PI);

            drive_commands.push_back(turn_command);
        }
    } 

    ROS_DEBUG("initialized commands.");

    while(ros::ok()) {
        ROS_DEBUG("%d and %d", command_done(), !drive_commands.empty());

        if (command_done() && !drive_commands.empty()){
            ROS_INFO("Command done. %ld commands left.", drive_commands.size());
            current_command = drive_commands.front();
            drive_commands.pop_front();

            current_did[0] = 0.0;
            current_did[1] = 0.0;

            speed_left = 0.0;
            speed_right = 0.0;

            left_encoder_before_command = left_encoder_current;
            right_encoder_before_command = right_encoder_current;
        }

        drive(diff_drive, speed_left, speed_right);

        ROS_INFO("command: (l=%f, r=%f) | did: (l=%f, r=%f) | speed: (l=%f, r=%f)", 
            current_command[0], current_command[1], 
            current_did[0], current_did[1],
            speed_left, speed_right);

        signal(SIGINT, stop_driving);
        ros::spinOnce();
    }

    return 0;
}

