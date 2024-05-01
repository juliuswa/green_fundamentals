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

// int main(int argc, char **argv)
// {
//     ros::init(argc, argv, "square_no_sensors");
//     ros::NodeHandle n;
//     ros::ServiceClient diffDrive = n.serviceClient<create_fundamentals::DiffDrive>("diff_drive");

//     for(int i = 0; i < 5; i++) {
// 	    for(int i = 0; i < 4; i++) {
//         	drive(diffDrive, 8, 8);
//         	ros::Duration(4.0).sleep();
//         	drive(diffDrive, 10, -10);
// 		    ros::Duration(0.62).sleep();
//     	}
//     }
//     stopDriving();
//     return 0;
// }

int main(int argc, char **argv) {
    ros::init(argc, argv, "square_no_sensors");
    ros::NodeHandle n;
    ros::ServiceClient diffDrive = n.serviceClient<create_fundamentals::DiffDrive>("diff_drive");
    
    float wheel_speed = 10.;
    float wheel_radius = 0.036;
    float wheelbase = 0.235;
    float s = 1.;

    float D = M_PI * wheelbase / 4;

    float rad_per_s = ((D/s)/wheel_radius);

    drive(diffDrive, -rad_per_s, rad_per_s);
    ros::Duration(s).sleep();
    stopDriving();
}