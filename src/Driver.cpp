#include "Driver.h"

void Driver::Driver()
{
    ros::NodeHandle n;

    ros::Subscriber sensorSub = n.subscribe("sensorPacket", 1, calculate_wheel_speeds);
    diff_drive = n.serviceClient<create_fundamentals::DiffDrive>("diff_drive");

    current_command.push_back(0.0);
    current_command.push_back(0.0);

    current_did.push_back(0.0);
    current_did.push_back(0.0);
}

void Driver::drive(ros::ServiceClient& diffDrive, int left, int right) {
    create_fundamentals::DiffDrive srv;
    srv.request.left = left;
    srv.request.right = right;
    diffDrive.call(srv);
}

bool Driver::command_done() {
    return std::abs(current_command[0] - current_did[0]) < tolerance
           && std::abs(current_command[1] - current_did[1]) < tolerance
           && !(std::abs(speed_left) > 1.0)
           && !(std::abs(speed_right) > 1.0);
}

float Driver::calculate_speed(float delta, float velocity) {
    float p_control = spring_constant * delta;
    float pd_control = p_control - damping_constant * velocity;
    return std::round(pd_control);
}

void Driver::calculate_wheel_speeds(const create_fundamentals::SensorPacket::ConstPtr& sensor_packet) {
    ROS_DEBUG("calculate_wheel_speeds");
    speed_left = calculate_speed(current_command[0] - current_did[0], speed_left);
    speed_right = calculate_speed(current_command[1] - current_did[1], speed_right);

    left_encoder_current = sensor_packet->encoderLeft;
    right_encoder_current = sensor_packet->encoderRight;

    current_did[0] = left_encoder_current - left_encoder_before_command;
    current_did[1] = right_encoder_current - right_encoder_before_command;
}

void Driver::execute_command(std::vector<float>& command) {
    ROS_INFO("DRIVER: new command [%f, %f]", command[0], command[1]);
    current_command = command;

    left_encoder_before_command = left_encoder_current;
    right_encoder_before_command = right_encoder_current;

    current_did[0] = 0.0;
    current_did[1] = 0.0;

    while(!command_done()) {
        drive(diff_drive, speed_left, speed_right);

        ROS_DEBUG("command: (l=%f, r=%f) | did: (l=%f, r=%f) | speed: (l=%f, r=%f)",
                 current_command[0], current_command[1],
                 current_did[0], current_did[1],
                 speed_left, speed_right);

        ros::spinOnce();
    }

    speed_left = 0.0;
    speed_right = 0.0;

    return;
}