#include "ros/ros.h"
#include <boost/bind.hpp>
#include <cmath>

#include "green_fundamentals/GoToGoal.h"

#define DISTANCE_EPS 0.025
#define KB -1.5
#define KP 3
#define KA 8

bool isGoalSet = false;

float current_x = 0.;
float current_y = 0.;
float current_theta = 0.;

float goal_x = 0.;
float goal_y = 0.;
float goal_theta = 0.;

void set_current_position(const green_fundamentals::Position::ConstPtr& msg) {
    current_x = msg->x;
    current_y = msg->y;
    current_theta = msg->theta;
}

bool set_goal(green_fundamentals::GoToGoal::Request req, green_fundamentals::GoToGoal::Response res) {
   if (!isGoalSet) {
    goal_x = req.x;
    goal_y = req.y;
    goal_theta = req.theta;
    isGoalSet = true;
    res.success = true;
    return true;
   } else {
    res.success = false;
    return false;
   }
}

float normalize_half_pi(float alpha){
    alpha = normalize_pi(alpha);
    if (alpha > M_PI/2) return alpha - M_PI;
    else if (alpha < -M_PI/2) return alpha + M_PI;
    else return alpha;
}

float normalize_pi(float alpha){
    while (alpha > M_PI) {
        alpha -= 2*M_PI;
    }
        
    while (alpha < -M_PI) {
        alpha += 2*M_PI;
    }
       
    return alpha;
}

float sign(float x){
    if (x >= 0) return 1;
    else return -1;
}

float get_goal_distance() {
    float diffX = current_x - goal_x;
    float diffY = current_y - goal_y;
    return std::sqrt(diffX*diffX + diffY*diffY)
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "go_to_goal_node");
    ros::NodeHandle n;

    ros::ServiceClient diffDriveSrv = n.serviceClient<create_fundamentals::DiffDrive>("diff_drive");

    ros::Subscriber sub = n.subscribe("odometry", 1, set_current_position);
    ros::ServiceServer service = n.advertiseService("set_goal", set_goal);

    ros::Rate loop_rate(10);

    while (ros::ok()) {
        float v = 0.;
        float w = 0.;

        if (isGoalSet) {
            // 1. Drive to point

            // 2. Drehe dich in den richtigen Winkel

            
            // calulate difference between current position and goal
            float goal_heading = std::atan2(goal_y - current_y, goal_x - current_x);
            float a = goal_heading - current_theta;

            float theta = normalize_pi(current_theta - goal_theta)
            float b = -theta - a;
            float d = get_goal_distance();

            direction = sign(std::cos(a));
            a = normalize_half_pi(a);
            b = normalize_half_pi(b);

            // calculate wheel velocities
            if (fabs(d) < DISTANCE_EPS) {
                // Only turn
                v = 0.;
                w = theta * KB;
            } else {
                v = KP * d * direction;
                w = KA * a + KB * b;
            }

            // send wheel velocities


        }

        loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}