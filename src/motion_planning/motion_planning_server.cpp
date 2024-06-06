#include <cstdlib>
#include "ros/ros.h"
#include "../Eigen/Dense"
#include <random>

#include "nav_msgs/OccupancyGrid.h"
#include "green_fundamentals/Position.h"
#include "green_fundamentals/Cell.h"
#include "green_fundamentals/Grid.h"
#include "green_fundamentals/Obstacle.h"
#include "green_fundamentals/DriveTo.h"
#include "../robot_constants.h"

green_fundamentals::Position position;
green_fundamentals::Position target_position;

bool aligned = false;
bool obstacle = false;

int mode = 0;

// Map data
ros::Subscriber grid_map_sub;
bool map_received = false;
std::vector<green_fundamentals::Cell> flat_map;

ros::ServiceClient driving_service;

void map_callback(const green_fundamentals::Grid::ConstPtr& msg)
{     
    for (const auto& row : msg->rows)
    {
        for (const auto& cell : row.cells)
        {
            flat_map.push_back(cell);
        }
    }

    grid_map_sub.shutdown();
    map_received = true;
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

std::pair<int, int> get_current_cell() {
    std::pair<int, int> cell {floor(position.x / CELL_LENGTH), floor(position.y / CELL_LENGTH)};
    return cell;
}

green_fundamentals::Position get_cell_midpoint(std::pair<int, int> cell_idx) {
    float cell_mid_x = cell_idx.first * CELL_LENGTH + CELL_LENGTH / 2;
    float cell_mid_y = cell_idx.second * CELL_LENGTH + CELL_LENGTH / 2;
    
    green_fundamentals::Position position;
    position.x = cell_mid_x;
    position.y = cell_mid_y;
    position.theta = 0.0;

    return position;
}

green_fundamentals::Position get_align_position() {
    ROS_DEBUG("position: (%f, %f) th: %f", position.x, position.y, position.theta);

    std::pair<int, int> cell = get_current_cell();

    green_fundamentals::Position position = get_cell_midpoint(cell);

    float theta_delta = floor(position.theta / (M_PI / 2)) * (M_PI / 2);
    //ROS_DEBUG("global_theta: %f deg.", theta_delta * 180 / M_PI);
    position.theta = theta_delta;

    return position;
}

bool is_same_position(green_fundamentals::Position p1,  green_fundamentals::Position p2) {
    if (p1 != p1 || p2 != p2) {
        return false;
    }

    return abs(p1.x - p2.x) < POS_EPSILON && abs(p1.y - p2.y) < POS_EPSILON;
}

green_fundamentals::Position get_neighbor_cell_midpoint() {
    std::pair<int, int> cell_idx = get_current_cell();
    ROS_DEBUG("current cell: (%d, %d)", cell_idx.first, cell_idx.second);
    
    //TODO: remove hardcoded "3"
    green_fundamentals::Cell cell = flat_map[cell_idx.first + 3 * (2 - cell_idx.second)];

    std::default_random_engine generator;
    std::uniform_real_distribution<float> uni_dist(0., 1.);

    int direction;

    for (int i = 0; i < 4; i++) {
        bool is_wall = false;

        for (int j = 0; j < cell.walls.size(); j++) {
            if (cell.walls[j] == i) {
                is_wall = true;
            }
        }

        if (!is_wall) {
            direction = i;
            break;
        }
    }

    ROS_DEBUG("Direction: %d", direction);

    if (direction == 0) {      
        cell_idx.first += 1;  
    }
    else if (direction == 1) {  
        cell_idx.second += 1;
    }
    else if (direction == 2) {
        cell_idx.first -= 1;       
    }
    else if (direction == 3) { 
        cell_idx.second -= 1;   
    }

    ROS_DEBUG("neighbor cell: (%d, %d)", cell_idx.first, cell_idx.second);
    return get_cell_midpoint(cell_idx);
}

green_fundamentals::Position get_target() {
    if (mode == 0) { //align
        return get_align_position();
    }
    if (mode == 1) {
        mode == 2;
        return get_neighbor_cell_midpoint();
    }   
        
    return target_position;
}

void main_loop() {
    ros::Rate loop_rate(30);
    while(ros::ok()) {
        ros::spinOnce();

        if (position != position) {
            ROS_INFO("No position.");
            continue;
        }
            
        target_position = get_target();
        ROS_INFO("target_position: (%f, %f), th: %f", target_position.x, target_position.y, target_position.theta);

        if (is_same_position(position, target_position)) {
            mode = 1;
            continue;
        }

        if (!obstacle) {            
            call_drive_to_service(position, target_position, mode == 0);
        }
        else {
            green_fundamentals::Position turned_position;
            turned_position.x = position.x;
            turned_position.y = position.y;
            turned_position.theta = position.theta + 0.5;
            call_drive_to_service(position, turned_position, true);
        }
        
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

    grid_map_sub = n.subscribe("grid_map", 1, map_callback);
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
