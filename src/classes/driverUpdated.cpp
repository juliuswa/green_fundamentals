#include "driverUpdated.h"

void Driver::drive(wheelCommand& command) {
    create_fundamentals::DiffDrive srv;
    srv.request.left = command.left_wheel;
    srv.request.right = command.right_wheel;
    diff_drive.call(srv);
}

void Driver::get_cur_position(const create_fundamentals::SensorPacket::ConstPtr& odometry) {
    cur_pos = odometry->pos;
}

bool Driver::arrived(Eigen::Vector2f& dest) {
    return (std::abs(cur_pos.pos_x - dest.x) + std::abs(cur_pos.pos_y - dest.y)) < 0.001;
}

void Driver::go_to_destination(Eigen::Vector2f& dest, float max_speed) {
    while(!arrived(dest)) { //FIXME
        
        float angle_to_dest = math.atan2(dest.y - cur_pos.pos_x, dest.x - cur_pos.pos_y); // in rad
        float angle = angle_to_dest - cur_pos.theta;
        float angle = fmod(angle + M_PI, 2 * M_PI) - M_PI;  // norm it so it's between -180° and 180°

        float percent = angle / M_PI / 4;
        float factor = -1 * (percent - 1);  // this will give 0 for 45°, -1 for 90° and 1 for 0°

        if(angle < 0) {  // decide which wheel is the inner wheel
            wheelCommand command = {max_speed, factor * max_speed};
        } else {
            wheelCommand command = {factor * max_speed, max_speed};
        }
        
        drive(command);
        ros::spinOnce();
    }
    
}

Driver::Driver(ros::NodeHandle& nh)
{
    diff_drive = nh.serviceClient<create_fundamentals::DiffDrive>("diff_drive");
    
    // ros::Subscriber odometrySub = n.subscribe("odometry", 1, &Driver::get_cur_position, &driver);  // FIXME to correct file
}