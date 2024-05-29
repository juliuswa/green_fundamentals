#include "ros/ros.h"
#include "../robot_constants.h"
#include "../Eigen/Dense"
#include "green_fundamentals/ExecutePlan.h"
#include "green_fundamentals/Position.h"

bool execute_plan(green_fundamentals::ExecutePlan::Request& req, green_fundamentals::ExecutePlan::Response& res) {
    std::vector<int> plan = req.plan; 
    ROS_INFO("Plan Size %li", plan.size());

    for(int i = 0; i < plan.size(); i++){
        Eigen::Vector2f target {0, 0};

        ROS_INFO("COMMAND %i", plan[i]);
        switch (plan[i]) {
            case 0:  // RIGHT
                target[1] -= cell_length;
                break;
            case 1:  // UP
                target[0] += cell_length;
                break;
            case 2:  // LEFT
                target[1] += cell_length;
                break;
            case 3:  // DOWN
                target[0] -= cell_length;
                break;
            default:
                ROS_ERROR("invalid command in plan");
                break;
        } 
    }
    return true;  // TODO: change to return plan array with points
}

int main(int argc, char **argv)
{  
    ros::init(argc, argv, "execute_plan_server");
    ros::NodeHandle n;

    ros::ServiceServer service = n.advertiseService("execute_plan", execute_plan);

    ROS_INFO("Ready to execute_plan");
    ros::spin();

    return 0;
}