#include "ros/ros.h"
#include "../Eigen/Dense"
#include "green_fundamentals/ExecutePlan.h"
#include "green_fundamentals/DetectGrid.h"
#include "green_fundamentals/DriveTo.h"
#include "create_fundamentals/SensorPacket.h"
#include "create_fundamentals/DiffDrive.h"
#include "green_fundamentals/Position.h"
#include "green_fundamentals/DriveToWaypoints.h"

// current position
float x_position;
float y_position;
float theta_position;

float cell_length = 0.78;
Eigen::Vector2f up;
Eigen::Vector2f down;
Eigen::Vector2f right;
Eigen::Vector2f left;

std::vector<Eigen::Vector2f> obstacles;
std::vector<Eigen::Vector2f> waypoints;

std::vector<float> xs, ys, xs_obst, ys_obst;

bool obstacle_detected = false;
ros::ServiceClient diff_drive;
ros::ServiceClient grid_detection_service;
ros::ServiceClient drive_to_waypoints_service;

void update_position(const green_fundamentals::Position::ConstPtr& position) {
    x_position = position->x;
    y_position = position->y;
    theta_position = position->theta;
}

void get_obstacles(Eigen::Vector2f target) {
    xs_obst.push_back(target[0] + up[0] * (cell_length / 2));
    ys_obst.push_back(target[1] + up[1] * (cell_length / 2));

    xs_obst.push_back(target[0] + down[0] * (cell_length / 2));
    ys_obst.push_back(target[1] + down[1] * (cell_length / 2));

    xs_obst.push_back(target[0] + right[0] * (cell_length / 2));
    ys_obst.push_back(target[1] + right[1] * (cell_length / 2));

    xs_obst.push_back(target[0] + left[0] * (cell_length / 2));
    ys_obst.push_back(target[1] + left[1] * (cell_length / 2));
}
/*RIGHT = 0
int32 UP = 1
int32 LEFT = 2
int32 DOWN = 3*/
void get_waypoint(Eigen::Vector2f target, int command) {
    ROS_INFO("COMMAND %i", command);
    switch (command) {
        case 1:
            target[0] += up[0] * cell_length;
            target[1] += up[1] * cell_length;
            break;
        case 3:
            target[0] += down[0] * cell_length;
            target[1] += down[1] * cell_length;
            break;
        case 0:
            target[0] += right[0] * cell_length;
            target[1] += right[1] * cell_length;
            break;
        case 2:
            target[0] += left[0] * cell_length;
            target[1] += left[1] * cell_length;
            break;
        default:
            ROS_ERROR("invalid command in plan");
            break;
    } 
    xs.push_back(target[0]);
    ys.push_back(target[1]);
}

void detect_crash(const create_fundamentals::SensorPacket::ConstPtr& sensor_packet) {
    if(sensor_packet->bumpLeft || sensor_packet->bumpRight) {
        obstacle_detected = true;

        create_fundamentals::DiffDrive srv;
        srv.request.left = 0;
        srv.request.right = 0;
        diff_drive.call(srv);
    }
}

bool execute_plan(green_fundamentals::ExecutePlan::Request& req, green_fundamentals::ExecutePlan::Response& res) {
    std::vector<int> plan = req.plan; 
    ROS_INFO("Plan Size %i", plan.size());

    green_fundamentals::DetectGrid detect_grid_srv;
    green_fundamentals::DriveToWaypoints drive_to_waypoints_srv;

    for(int i = 0; i < plan.size(); i++){
        if(obstacle_detected) return false;

        Eigen::Vector2f target {x_position, y_position};

        if (grid_detection_service.call(detect_grid_srv)) {  // correct drifting
            target[0] += detect_grid_srv.response.x_offset;
            target[1] += detect_grid_srv.response.y_offset;

            ROS_INFO("grid position: x_offset=%f, y_offset=%f, theta_offset=%f deg.", 
            detect_grid_srv.response.x_offset, 
            detect_grid_srv.response.y_offset, 
            detect_grid_srv.response.theta_offset * 180 / M_PI);
        }

        // get the direction vectors according to robot after align, UP is where robot is looking
        up = {cos(theta_position), sin(theta_position)};
        down = {-up[0], -up[1]};
        right = {cos(theta_position + (M_PI/2)), sin(theta_position + (M_PI/2))};
        left = {cos(theta_position - (M_PI/2)), sin(theta_position - (M_PI/2))}; 

        get_waypoint(target, plan[i]);
        get_obstacles(target);
    }

    if(obstacle_detected) return false;

    drive_to_waypoints_srv.request.xs = xs;
    drive_to_waypoints_srv.request.ys = ys;
    drive_to_waypoints_srv.request.xs_obst = xs_obst;
    drive_to_waypoints_srv.request.ys_obst = ys_obst;

    if (!drive_to_waypoints_service.call(drive_to_waypoints_srv))
    {
        ROS_ERROR("failed to call driver_service");
    }

    return !obstacle_detected;
}

int main(int argc, char **argv)
{  
    ros::init(argc, argv, "execute_plan_server");
    ros::NodeHandle n;

    ros::ServiceServer service = n.advertiseService("execute_plan", execute_plan);
    grid_detection_service = n.serviceClient<green_fundamentals::DetectGrid>("detect_grid");
    diff_drive = n.serviceClient<create_fundamentals::DiffDrive>("diff_drive");
    drive_to_waypoints_service = n.serviceClient<green_fundamentals::DriveToWaypoints>("drive_to_waypoints");

    ros::Subscriber odometry_sub = n.subscribe("odometry", 1, update_position);
    ros::Subscriber sensor_sub = n.subscribe("sensor_packet", 1, detect_crash);

    ROS_INFO("Ready to execute_plan");
    ros::spin();

    return 0;
}