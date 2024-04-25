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
    ros::init(argc, argv, "square_no_sensors");
    ros::NodeHandle n;
    ros::ServiceClient diffDrive = n.serviceClient<create_fundamentals::DiffDrive>("diff_drive");

    for(int i = 0; i < 4; i++) {
        drive(diffDrive, 5, 5);
        sleep(2);
        drive(diffDrive, 10, -10);
        sleep(0.8);
    }
    stopDriving();
    return 0;
}
