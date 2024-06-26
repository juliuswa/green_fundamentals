#include "ros/ros.h"
#include <boost/bind.hpp>
#include <cmath>
#include <deque>

#include "../Eigen/Dense"

#include "green_fundamentals/DriveToWaypoints.h"
#include "green_fundamentals/Velocities.h"
#include "green_fundamentals/Position.h"

bool is_waypoints_set = false;

std::deque<Eigen::Vector2f> waypoints;
std::deque<Eigen::Vector2f> obstacles;

Eigen::Vector2f current_goal;

float KP = 1.;
float KD = 0.1;
float KI = 0.0;
float I_err = 0.;
float err_prev = 0.;

std::string state = "Wait";

Eigen::Vector2f current_p {0., 0.};
float current_theta;

bool service_call(green_fundamentals::DriveToWaypoints::Request& req, green_fundamentals::DriveToWaypoints::Response& res)
{
    ROS_INFO("Received Call");
    if (is_waypoints_set) {
        res.success = false;
        return false;
    }

    // Set Waypoints
    for(int i = 0; i < req.xs.size(); i++) {
        float x = req.xs[i];
        float y = req.ys[i];
        
        Eigen::Vector2f p {x, y};

        waypoints.push_back(p);
    }

    // Set Obstacles
    for(int i = 0; i < req.xs_obst.size(); i++) {
        float x = req.xs_obst[i];
        float y = req.ys_obst[i];
        
        Eigen::Vector2f p {x, y};

        obstacles.push_back(p);
    }

    is_waypoints_set = true;

    res.success = true;
    ROS_INFO("New waypoints set");
    return true;
}

void odometry_call(const green_fundamentals::Position::ConstPtr& position) {
    current_p[0] = position->x;
    current_p[1]= position->y;
    current_theta = position->theta;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "controller_node");
  ros::NodeHandle n;

  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info)) {
    ros::console::notifyLoggerLevelsChanged();
  }

  ros::ServiceServer service = n.advertiseService("drive_to_waypoints", service_call);
  ros::Publisher cmd_pub = n.advertise<green_fundamentals::Velocities>("cmd_vel", 1);
  ros::Subscriber sub = n.subscribe("odometry", 1, odometry_call);

  green_fundamentals::Velocities msg;

  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    if (is_waypoints_set)
    {   
        float dist_error = (current_goal - current_p).norm();
        float theta_goal = atan2((current_goal - current_p)[1],(current_goal - current_p)[0]);
        float theta_error = theta_goal - current_theta;
        theta_error = atan2(sin(theta_error), cos(theta_error)); // Limits the error to (-pi, pi)
        
        float P = KP * theta_error;
        I_err += KI * theta_error;
        float D = KD * (theta_error - err_prev);
        err_prev = theta_error;

        ROS_INFO("Current Goal (%f, %f) Current Position (%f, %f)", current_goal[0], current_goal[1], current_p[0], current_p[1]);

        if (state == "Wait") {
            ROS_INFO("State = Wait");
            if (waypoints.size() > 0) {
                current_goal = waypoints.front();
                waypoints.pop_front();
                state = "Turn";
            } else {
                is_waypoints_set = false;
                waypoints.clear();
                obstacles.clear();
            }
            msg.v = 0;
            msg.w = 0;
            cmd_pub.publish(msg);
        }
        else if (state == "Turn") {
            ROS_INFO("State = Turn, theta error=%f", fabs(theta_error));
            if (fabs(theta_error) < 0.15) {
                state = "Drive";
                msg.v = 0;
                msg.w = 0;
                cmd_pub.publish(msg);
            } else {
                msg.v = 0;
                msg.w = P + D + I_err;
                cmd_pub.publish(msg);
            }
        }
        else if (state == "Drive") {
            ROS_INFO("State = Drive dist_error=%f", dist_error);
            if (dist_error < 0.1) {
                state = "Wait";
                msg.v = 0;
                msg.w = 0;
                cmd_pub.publish(msg);
            }
            else {
                msg.v = KP * dist_error;
                msg.w = P + D + I_err;
                cmd_pub.publish(msg);
            }
        }
    }

    loop_rate.sleep();
    ros::spinOnce();
  }

  
  return 0;
}