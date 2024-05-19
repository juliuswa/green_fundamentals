#include "ros/ros.h"
#include <boost/bind.hpp>
#include <cmath>
#include "../Eigen/Dense"

#include "green_fundamentals/DriveToWaypoints.h"
#include "green_fundamentals/Velocities.h"
#include "green_fundamentals/Position.h"

bool is_waypoints_set = false;

std::vector<Eigen::Vector2f> waypoints;
std::vector<Eigen::Vector2f> obstacles;

float KX = 1.;
float KY = 1.;
float a = 0.05;
float rep_weight = 0.001;
float T = 100;

Eigen::Vector2f current_p {0., 0.};
float current_theta;

int counter = 0;

std::string state = "Turn";

Eigen::Vector2f potential_field()
{
    Eigen::Vector2f force {0., 0.};

    for (int i = 0; i < obstacles.size(); i++) {
        float dist = (obstacles[i] - current_p).norm();
        if (dist == 0) dist = 0.00001;
        float repulsive_force = rep_weight/pow(dist, 2);
        force += repulsive_force * (current_p - obstacles[i]) / dist;
    }

    return force;
}

float binom(int n, int k) {
    if (k > n - k) {
        k = n - k; // Since C(n, k) == C(n, n-k)
    }
    float res = 1.;
    for (int i = 0; i < k; ++i) {
        res *= (n - i);
        res /= (i + 1);
    }
    return res;
}

Eigen::Vector2f bezier_curve(float t)
{
    Eigen::Vector2f b {0., 0.};
    for (int i = 0; i < waypoints.size(); i++){
        Eigen::Vector2f p = waypoints[i];
        int n = waypoints.size()-1;
        b += (binom(n, i) * pow(t, i) * pow(1-t, n-i)) * p;
    }
    
    return b;
}

std::pair<Eigen::Vector2f, Eigen::Vector2f> trajectory(int t)
{
    Eigen::Vector2f p = bezier_curve(t/T);
    return {p, Eigen::Vector2f {0., 0.}};
}

bool service_call(green_fundamentals::DriveToWaypoints::Request& req, green_fundamentals::DriveToWaypoints::Response& res)
{
    if (is_waypoints_set) {
        res.success = false;
        return false;
    }

    waypoints.push_back(current_p);

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
    return true;
}

void odometry_call(const green_fundamentals::Position::ConstPtr& position) {
    current_p[0] = position->x;
    current_p[0]= position->y;
    current_theta = position->theta;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "smooth_controller_node");
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
        if (state == "Turn") {
            std::pair<Eigen::Vector2f, Eigen::Vector2f> goal = trajectory(counter);
            float theta_goal = atan2((goal.first - current_p)[1],(goal.first - current_p)[0]);
            float theta_error = theta_goal - current_theta;
            theta_error = atan2(sin(theta_error), cos(theta_error));

            ROS_INFO("State = Turn");
            if (fabs(theta_error) < 0.05) {
                state = "Drive";
                msg.v = 0;
                msg.w = 0;
                cmd_pub.publish(msg);
            } else {
                msg.v = 0;
                msg.w = theta_error;
                cmd_pub.publish(msg);
            }
        } else {
            ROS_INFO("State = Drive");
            // Check if last point is reached
            float dist = (waypoints[waypoints.size()-1] - current_p).norm();
            if (dist < 0.05) {
                msg.v = 0;
                msg.w = 0;
                cmd_pub.publish(msg);
                is_waypoints_set = false;
                waypoints.clear();
                obstacles.clear();
            }
            else {
                // Drive
                std::pair<Eigen::Vector2f, Eigen::Vector2f> goal = trajectory(counter);

                Eigen::Vector2f p_err = goal.first - current_p;

                ROS_INFO("POS: %f, %f   GOAL: %f, %f", current_p[0], current_p[1], goal.first[0], goal.first[1]);

                Eigen::Vector2f rep_forces = potential_field();

                goal.second += rep_forces;

                msg.v = cos(current_theta)*(goal.second[0] + KX * p_err[0]) + sin(current_theta)*(goal.second[1] + KY * p_err[1]);
                msg.w = -(1/a)*sin(current_theta)*(goal.second[0] + KX * p_err[0]) + (1/a)*cos(current_theta)*(goal.second[1] + KY * p_err[1]);

                counter++;
                cmd_pub.publish(msg);
            }
        }
    }

    loop_rate.sleep();
    ros::spinOnce();
  }

  
  return 0;
}