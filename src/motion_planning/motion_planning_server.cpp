#include <cstdlib>
#include "ros/ros.h"
#include "../Eigen/Dense"

#include "nav_msgs/OccupancyGrid.h"
#include "green_fundamentals/Position.h"
#include "green_fundamentals/Obstacle.h"
#include "green_fundamentals/DriveTo.h"
#include "../robot_constants.h"

green_fundamentals::Position position;

green_fundamentals::Position current_target;
green_fundamentals::Position last_target;

bool aligned = false;
bool obstacle = false;

// Map data
ros::Subscriber map_sub;
bool map_received = false;
std::vector<std::vector<int8_t>> map_data;
int map_height, map_width;


ros::ServiceClient driving_service;

void map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{    
    map_height = msg->info.height;
    map_width = msg->info.width;

    // Resize the map_data to match the map dimensions
    map_data.resize(map_height);
    for (int i = 0; i < map_height; ++i) {
        map_data[i].resize(map_width);
    }

    // Fill the 2D array with the occupancy data
    for (int y = 0; y < map_height; ++y) {
        for (int x = 0; x < map_width; ++x) {
            // Calculate the index in the 1D data array
            int index = x + y * map_width;
            map_data[y][x] = msg->data[index];
        }
    }

    ROS_INFO("Map received");
    map_received = true;
    map_sub.shutdown();
}

void set_obstacle(const green_fundamentals::Obstacle::ConstPtr& obst) {    
    obstacle = obst->front;

    if (obstacle) {
        ROS_INFO("Obstacle detected.");
    }
}

void update_position(const green_fundamentals::Position& pos) {
    ROS_DEBUG("update position: (%f, %f) th: %f", position.x, position.y, position.theta);
    position = pos;
}

void call_drive_to_service(green_fundamentals::Position current, green_fundamentals::Position target, bool rotate) {
    green_fundamentals::DriveTo drive_to_srv;

    drive_to_srv.request.x_current = current.x;
    drive_to_srv.request.y_current = current.y;
    drive_to_srv.request.theta_current = current.theta;
    drive_to_srv.request.x_target = target.x;
    drive_to_srv.request.y_target = target.y;
    drive_to_srv.request.theta_target = target.theta;
    drive_to_srv.request.rotate = rotate;

    ROS_INFO("Calling drive_to:\n\tcurrent: (%f, %f) th: %f\n\ttarget: (%f, %f) th: %f\n\trotate: %d", 
        current.x, 
        current.y, 
        current.theta,
        target.x, 
        target.y, 
        target.theta,
        rotate
        );

    if (!driving_service.call(drive_to_srv))
    {
        ROS_INFO("failed to call driver_service");
    }
}

green_fundamentals::Position get_align_position() {
    ROS_DEBUG("position: (%f, %f) th: %f", position.x, position.y, position.theta);

    int cell_x = floor(position.x / CELL_LENGTH);
    int cell_y = floor(position.y / CELL_LENGTH);
    ROS_DEBUG("cell: (%d, %d)", cell_x, cell_y);

    float cell_mid_x = cell_x * CELL_LENGTH + CELL_LENGTH / 2;
    float cell_mid_y = cell_y * CELL_LENGTH + CELL_LENGTH / 2;
    //ROS_DEBUG("cell_mid: (%f, %f)", cell_mid_x, cell_mid_y);

    float theta_delta = position.theta - floor(position.theta / (M_PI / 2)) * (M_PI / 2);
    //ROS_DEBUG("global_theta: %f deg.", theta_delta * 180 / M_PI);

    green_fundamentals::Position position;
    position.x = cell_mid_x;
    position.y = cell_mid_y;
    position.theta = theta_delta;

    return position;
}

bool is_same_position(green_fundamentals::Position p1,  green_fundamentals::Position p2) {
    if (p1 != p1 || p2 != p2) {
        return false;
    }

    return abs(p1.x - p2.x) < POS_EPSILON && abs(p1.y - p2.y) < POS_EPSILON;
}

void main_loop() {
    green_fundamentals::Position last_position;
    green_fundamentals::Position target_position;

    ros::Rate loop_rate(30);
    while(ros::ok()) {
        ros::spinOnce();

        if (position != position) {
            ROS_INFO("No position.");
            continue;
        }
        
        last_position = target_position;

        if (!aligned) {
            target_position = get_align_position();
        }

        ROS_INFO("target_position: (%f, %f), th: %f", target_position.x, target_position.y, target_position.theta);

        if (is_same_position(position, target_position)) {
            aligned = true;
            continue;
        }

        call_drive_to_service(position, target_position, true);

        loop_rate.sleep();
    }    
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "motion_planning_server");
    ros::NodeHandle n;
    
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
        ros::console::notifyLoggerLevelsChanged();
    }

    map_sub = n.subscribe("map", 1, map_callback);
    ros::Subscriber obstacle_sub = n.subscribe("obstacle", 1, set_obstacle);

    ros::Rate map_rate(10);
    while (!map_received)
    {
        ros::spinOnce();

        map_rate.sleep();
    }

    ROS_INFO("Map received.");

    ros::Subscriber position_sub = n.subscribe("position", 1, update_position);

    driving_service = n.serviceClient<green_fundamentals::DriveTo>("drive_to");

    main_loop();

    return 0;    
}
