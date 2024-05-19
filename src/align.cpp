#include <cstdlib>
#include <signal.h>
#include <deque>

#include "ros/ros.h"
#include "Eigen/Dense"
#include "sensor_msgs/LaserScan.h"
#include "create_fundamentals/DiffDrive.h"
#include "green_fundamentals/Position.h"
#include "green_fundamentals/DetectGrid.h"
#include "green_fundamentals/DriveTo.h"
#include "classes/driver.h"

float x_position;
float y_position;
float theta_position;

ros::ServiceClient driving_service;

void update_position(const green_fundamentals::Position::ConstPtr& position) {
    x_position = position->x;
    y_position = position->y;
    theta_position = position->theta;
}

void call_drive_to_service(float x, float y, float theta, bool rotate) {
    green_fundamentals::DriveTo drive_to_srv;

    drive_to_srv.request.x_target = x;
    drive_to_srv.request.y_target = y;
    drive_to_srv.request.rotate = rotate;
    drive_to_srv.request.theta_target = theta;

    ROS_INFO("Calling drive_to with (%f, %f, %d, %f)", x, y, rotate, theta);

    if (!driving_service.call(drive_to_srv))
    {
        ROS_INFO("failed to call driver_service");
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "align");
    ros::NodeHandle n;

    ros::Subscriber odometry_sub = n.subscribe("odometry", 1, update_position);
    ros::ServiceClient grid_detection_service = n.serviceClient<green_fundamentals::DetectGrid>("detect_grid");
    driving_service = n.serviceClient<green_fundamentals::DriveTo>("drive_to");

    int retry_count = 0;

    while (retry_count < 4) {
        ros::spinOnce();

        green_fundamentals::DetectGrid detect_grid_srv;

        if (!grid_detection_service.call(detect_grid_srv))
        {
            ROS_INFO("detecting grid failed.");    

            float new_theta = theta_position + M_PI / 2;
            new_theta = new_theta - floor(new_theta / M_PI * 2) *  M_PI * 2;

            call_drive_to_service(x_position, y_position, new_theta, true);
            ros::Duration(1).sleep();

            retry_count += 1;
            continue;
        }

        ROS_INFO("grid position: x_offset=%f, y_offset=%f, theta_offset=%f deg.", 
            detect_grid_srv.response.x_offset, 
            detect_grid_srv.response.y_offset, 
            detect_grid_srv.response.theta_offset * 180 / M_PI);   

        float x_offset = detect_grid_srv.response.x_offset;
        float y_offset = detect_grid_srv.response.y_offset;
        float theta_offset = detect_grid_srv.response.theta_offset;

        float x_global_offset = cos(theta_position) * x_offset - sin(theta_position) * y_offset;
        float y_global_offset = sin(theta_position) * x_offset + cos(theta_position) * y_offset;
        float theta_global_offset = theta_position + theta_offset;
        
        call_drive_to_service(x_global_offset, y_global_offset, theta_global_offset, true);
        break;
    }

    return 0;
}
