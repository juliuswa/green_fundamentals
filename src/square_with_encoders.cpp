#include "ros/ros.h"
#include <cstdlib>
#include "sensor_msgs/LaserScan.h"
#include <math.h>
#include "create_fundamentals/DiffDrive.h"
#include "create_fundamentals/ResetEncoders.h"
#include "create_fundamentals/SensorPacket.h"
#include <signal.h>

void drive (ros::ServiceClient& diffDrive, int left, int right) {
    create_fundamentals::DiffDrive srv;
    srv.request.left = left;
    srv.request.right = right;
    diffDrive.call(srv);
}

void send_vw_to_vl_vr(ros::ServiceClient& diffDrive, float v, float w) {

    float wheelbase = 0.235;

    float left = v + (w*wheelbase)/2;
    float right = v - (w*wheelbase)/2;

    create_fundamentals::DiffDrive srv;
    srv.request.left = left;
    srv.request.right = right;
    diffDrive.call(srv);
}

void stopDriving() {
    ros::NodeHandle n;
    ros::ServiceClient diffDrive = n.serviceClient<create_fundamentals::DiffDrive>("diff_drive");
    drive(diffDrive, 0, 0);
    ros::shutdown();
}

void reset_encoders(ros::ServiceClient& client) {
    create_fundamentals::ResetEncoders srv;
    client.call(srv);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "square_with_sensors");
    ros::NodeHandle n;
    ros::ServiceClient diffDrive = n.serviceClient<create_fundamentals::DiffDrive>("diff_drive");
    ros::ServiceClient encoders_client = n.serviceClient<create_fundamentals::ResetEncoders>("reset_encoders");

    reset_encoders(encoders_client);
    
    float wheel_radius = 0.036;
    float wheelbase = 0.235;
    float s = 1.;

    float D = M_PI * wheelbase / 4;

    float rad_per_s = ((D/s)/wheel_radius);

    drive(diffDrive, -rad_per_s, rad_per_s);
    ROS_INFO("Soll rad_per_s: %f, time in s: %f", rad_per_s, s);
    ros::Duration(s).sleep();
    stopDriving();
}

