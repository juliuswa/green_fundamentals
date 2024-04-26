#include "ros/ros.h"
#include <cstdlib>
#include "sensor_msgs/LaserScan.h"
#include <math.h>
#include "create_fundamentals/DiffDrive.h"
#include "create_fundamentals/SensorPacket.h"
#include <signal.h>

void drive (ros::ServiceClient& diffDrive, int left, int right) {
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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "square_with_encoders");
    ros::NodeHandle n;
    ros::ServiceClient diffDrive = n.serviceClient<create_fundamentals::DiffDrive>("diff_drive");

    ros::ServiceClient SensorPacket = n.serviceClient<create_fundamentals::SensorPacket>("sensor_packet");
    create_fundamentals::SensorPacket srv_sen;
    ROS_INFO("distance: %d, angle: %d", srv_sen.response.distance, srv_sen.response.angle);

    //for(int i = 0; i < 5; i++) {
        for(int i = 0; i < 4; i++) {
            drive(diffDrive, 8, 8);
            ROS_INFO("distance: %d, angle: %d", srv_sen.response.distance, srv_sen.response.angle);
            ros::Duration(4.0).sleep();
            drive(diffDrive, 10, -10);
            ROS_INFO("distance: %d, angle: %d", srv_sen.response.distance, srv_sen.response.angle);
            ros::Duration(0.62).sleep();
        }
    //}
    stopDriving();
    return 0;
}

