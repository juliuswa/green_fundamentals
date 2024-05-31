#include "ros/ros.h"
#include "../robot_constants.h"
#include "../Eigen/Dense"
#include "green_fundamentals/ExecutePlan.h"
#include "green_fundamentals/Vector2f.h"

bool drive_path(green_fundamentals::ExecutePlan::Request& req, green_fundamentals::ExecutePlan::Response& res) {
    std::vector<int> plan = req.plan; 
    ROS_INFO("Plan Size %li", plan.size());

    green_fundamentals::Vector2f target_old;
    target_old.x = 0;
    target_old.y = 0;
    res.success = true;

    for(int i = 0; i < plan.size(); i++){
        green_fundamentals::Vector2f target;

        ROS_INFO("COMMAND %i", plan[i]);
        switch (plan[i]) {
            case 0:  // RIGHT
                target.x = target_old.x;
                target.y = target_old.y - CELL_LENGTH;
                break;
            case 1:  // UP
                target.x = target_old.x + CELL_LENGTH;
                target.y = target_old.y;
                break;
            case 2:  // LEFT
                target.x = target_old.x;
                target.y = target_old.y + CELL_LENGTH;
                break;
            case 3:  // DOWN
                target.x = target_old.x - CELL_LENGTH;
                target.y = target_old.y;
                break;
            default:
                res.success = false;
                ROS_ERROR("invalid command in plan");
                break;
        }
        res.points.push_back(target);
        target_old.x = target.x;
        target_old.y = target.y;
    }
    return res.success;
}

int main(int argc, char **argv)
{  
    ros::init(argc, argv, "execute_plan_server");
    ros::NodeHandle n;

    ros::ServiceServer service = n.advertiseService("drive_path", drive_path);

    ROS_INFO("Ready to execute_plan");
    ros::spin();

    return 0;
}