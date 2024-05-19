#include "ros/ros.h"
#include <cmath>

#include "../robot_constants.h"
#include "green_fundamentals/Velocities.h"
#include "create_fundamentals/DiffDrive.h"

ros::ServiceClient diff_drive_client;
create_fundamentals::DiffDrive srv;

void callback(const green_fundamentals::Velocities::ConstPtr& msg) {

    float v = msg->v;
    float w = msg->w;

    float leftVel = (v - WHEEL_BASE * w / 2) / (WHEEL_RADIUS);
    float rightVel = (v + WHEEL_BASE * w / 2) / (WHEEL_RADIUS);

    if (fabs(leftVel) > MAX_VELOCITY || fabs(rightVel) > MAX_VELOCITY)
    {
        float factor = MAX_VELOCITY/std::max(leftVel, rightVel);
        leftVel *= factor;
        rightVel *= factor;
    }

    ROS_INFO("Sending wl=%f wr=%f", leftVel, rightVel);
    srv.request.left = leftVel;
    srv.request.right = rightVel;
    diff_drive_client.call(srv);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "cmd_vel_node");
  ros::NodeHandle n;

  diff_drive_client = n.serviceClient<create_fundamentals::DiffDrive>("diff_drive");
  ros::Subscriber sub = n.subscribe("cmd_vel", 1, callback);

  ros::spin();
  return 0;
}

