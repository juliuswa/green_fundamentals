#include "ros/ros.h"
#include "std_srvs/Empty.h"
#include "nav_msgs/OccupancyGrid.h"
#include <deque>

#include "green_fundamentals/Position.h"
#include "green_fundamentals/Grid.h"
#include "green_fundamentals/Cell.h"
#include "green_fundamentals/Pose.h"
#include "green_fundamentals/DriveTo.h"
#include "green_fundamentals/ExecutePlan.h"
#include "robot_constants.h"

#define REASONABLE_DISTANCE 0.3

enum State {
    INIT,
    LOCALIZE,
    ALIGN,
    IDLE,
    EXECUTE_PLAN
};

State state = State::INIT;

State last_state = state;

enum Orientation {
    LEFT,
    RIGHT,
    UP,
    DOWN
};

struct Position {
    float x, y, theta;
    int row, col;
    Orientation orientation = Orientation::RIGHT;
};

bool is_localized = false;
int localization_count = 0;

struct Target {
    float x, y, theta;
    bool should_rotate = false;
    bool must_be_reached = false;
};

Position my_position{0., 0., 0., 0, 0};
Position old_position{0., 0., 0., 0, 0};
bool is_first_position = true;
std::deque<Target> target_list;

// Map
struct Cell {
    float x, y;
    bool wall_left, wall_up, wall_right, wall_down;
};

ros::Subscriber map_sub;
bool map_received = false;
std::vector<std::vector<Cell>> cell_info;

// Publishers
ros::Publisher pose_pub;

// Clients
ros::ServiceClient start_localize_client, mover_set_idle_client, mover_set_wander_client, mover_drive_to_client;

void add_target_front(float x, float y, float theta, bool should_rotate, bool must_be_reached)
{
    Target target{x, y, theta, should_rotate, must_be_reached};
    target_list.push_front(target);
}

void add_target_back(float x, float y, float theta, bool should_rotate, bool must_be_reached)
{
    Target target{x, y, theta, should_rotate, must_be_reached};
    target_list.push_back(target);
}

void map_callback(const green_fundamentals::Grid::ConstPtr& msg)
{   
    ROS_DEBUG("Inside map_callback...");

    for (int row = 0; row < msg->rows.size(); row++)
    {
        std::vector<Cell> column_cells;

        for (int col = 0; col < msg->rows[row].cells.size(); col++)
        {
            Cell new_cell;
            new_cell.x = (float)row * CELL_LENGTH + (CELL_LENGTH / 2);
            new_cell.y = (float)row * CELL_LENGTH + (CELL_LENGTH / 2);
            
            green_fundamentals::Cell current = msg->rows[msg->rows.size() - row].cells[col];
            for (auto wall : current.walls) {
                switch(wall) {
                    case 0:
                       new_cell.wall_right = true;
                       break;
                    case 1:
                       new_cell.wall_up = true;
                       break; 
                    case 2:
                       new_cell.wall_left = true;
                       break; 
                    case 3:
                       new_cell.wall_down = true;
                       break; 
                }
            }

            column_cells.push_back(new_cell);
        }

        cell_info.push_back(column_cells);
    }

    map_received = true;
    map_sub.shutdown();
}

void localization_callback(const green_fundamentals::Position::ConstPtr& msg)
{   
    if (is_first_position)
    {
        old_position.x = msg->x;
        old_position.y = msg->y;
        old_position.theta = msg->theta;
        old_position.row = floor(my_position.y / CELL_LENGTH);
        old_position.col = floor(my_position.x / CELL_LENGTH);
        is_first_position = false;
    }
    else
    {
        old_position.x = my_position.x;
        old_position.y = my_position.y;
        old_position.theta = my_position.theta;
        old_position.row = my_position.row;
        old_position.col = my_position.col;
    }

    my_position.x = msg->x;
    my_position.y = msg->y;
    my_position.theta = msg->theta;
    my_position.row = floor(my_position.y / CELL_LENGTH);
    my_position.col = floor(my_position.x / CELL_LENGTH);

    if (my_position.theta <= M_PI/4 && my_position.theta > -M_PI/4)
    {
        my_position.orientation = Orientation::RIGHT;
    }
    else if (my_position.theta > M_PI/4 && my_position.theta < 3*M_PI/4)
    {
        my_position.orientation = Orientation::UP;
    }
    else if (my_position.theta <= -3*M_PI/4 || my_position.theta >= 3*M_PI/4)
    {
        my_position.orientation = Orientation::LEFT;
    }
    else if (my_position.theta > -3*M_PI/4 && my_position.theta <= -M_PI/4)
    {
        my_position.orientation = Orientation::DOWN;
    }

    // Is localized?
    if (fabs(my_position.x - old_position.x) < REASONABLE_DISTANCE && fabs(my_position.y - old_position.y) < REASONABLE_DISTANCE) 
    {
        localization_count++;
    }
    else {
        localization_count = 0;
    }

    is_localized = localization_count > 15;

    if (is_localized)
    {   
        green_fundamentals::Pose pose;

        pose.row = my_position.row;
        pose.column = my_position.col;
        switch (my_position.orientation)
        {
        case Orientation::UP:
            pose.orientation = green_fundamentals::Pose::UP;
            break;
        case Orientation::DOWN:
            pose.orientation = green_fundamentals::Pose::DOWN;
            break;
        case Orientation::LEFT:
            pose.orientation = green_fundamentals::Pose::LEFT;
            break;
        case Orientation::RIGHT:
            pose.orientation = green_fundamentals::Pose::RIGHT;
            break;
        }

        pose_pub.publish(pose);
    }
    else {
        state = State::LOCALIZE;
    }
}

std::pair<float, float> get_current_cell_center()
{
    Cell cell = cell_info[my_position.row][my_position.col];
    return {cell.x, cell.y};
}

void set_target() {
    if (target_list.empty()) {
        ROS_DEBUG("Cannot set target, because target_list is empty");
        return;
    }

    green_fundamentals::DriveTo drive_to_msg;

    Target current_target = target_list.front();

    drive_to_msg.request.x_current = my_position.x;
    drive_to_msg.request.y_current = my_position.y;
    drive_to_msg.request.theta_current = my_position.theta;
    drive_to_msg.request.x_target = current_target.x;
    drive_to_msg.request.y_target = current_target.y;
    drive_to_msg.request.theta_target = current_target.theta;
    drive_to_msg.request.rotate = current_target.should_rotate;

    if (!mover_drive_to_client.call(drive_to_msg))
    {
        ROS_DEBUG("failed to call driver_service");
    }
}

bool current_target_reached()
{   
    if (target_list.empty()) {
        return true;
    }

    Target current_target = target_list.front();

    float pos_epsilon = current_target.must_be_reached ? POS_EPSILON : SOFT_EPSILON;

    bool distance_diff = fabs(my_position.x - current_target.x) < pos_epsilon && fabs(my_position.y - current_target.y) < pos_epsilon;
    bool angle_diff = fabs(my_position.theta - current_target.theta) < THETA_EPSILON;

    if (current_target.should_rotate) return distance_diff && angle_diff;
    
    return distance_diff;
}

bool set_execute_plan_callback(green_fundamentals::ExecutePlan::Request  &req, green_fundamentals::ExecutePlan::Response &res)
{
    target_list.clear();

    int row = my_position.row;
    int col = my_position.col;

    for (int i = 0; i < req.plan.size(); i++)
    {
        Target new_target;

        bool move_possible = true;

        switch (req.plan[i])
        {
        case  0: // RIGHT
            move_possible = !cell_info[row][col].wall_right;
            col++;
            break;
        case  1: // UP
            move_possible = !cell_info[row][col].wall_up;
            row++;
            break;
        case  2: // LEFT
            move_possible = !cell_info[row][col].wall_left;
            col--;
            break;
        case  3: // DOWN
            move_possible = !cell_info[row][col].wall_down;
            row--;
            break;
        }

        if (!move_possible || row > cell_info.size() || row < 0 || col > cell_info[0].size() || col < 0)
        {
            target_list.clear();
            res.success = false;
            state = State::IDLE;
            ROS_DEBUG("Plan cannot be fulfilled, because we would move out of the maze or into a wall.");
            ROS_INFO("State = IDLE");
            return false;
        }

        Cell center = cell_info[row][col];
        new_target.x = center.x;
        new_target.y = center.y;
        new_target.must_be_reached = i == req.plan.size()-1 ? true : false;
        target_list.push_back(new_target);
    }

    state = State::EXECUTE_PLAN;
    ROS_DEBUG("Added targets to target_list. Now we have %ld targets.", target_list.size());
    ROS_INFO("State = EXECUTE_PLAN");
    res.success = true;
    return true;
}

void execute_plan()
{
    if (target_list.empty())  {
        state = State::IDLE;
        ROS_INFO("Target list is empty");
        ROS_INFO("State = IDLE");
        return;
    }
    else if (current_target_reached())
    {
        target_list.pop_front();
        ROS_INFO("Next Target reached.");
    }
}

void localize()
{   
    // Wander
    std_srvs::Empty empty;
    mover_set_wander_client.call(empty);

    ros::Rate loop_rate(15);
    while (!is_localized)
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    state = State::ALIGN;
    target_list.clear();
}

void align()
{
    if (target_list.empty())
    {
        std::pair<float, float> target = get_current_cell_center(); // x, y
        ROS_DEBUG("Current cell is (%d, %d)", my_position.row, my_position.col);
        ROS_DEBUG("Cell Center is (%f, %f)", target.first, target.second);
        add_target_front(target.first, target.second, M_PI/2, true, true);
        set_target();
    }
    
    if (current_target_reached())
    {
        state = State::IDLE;
        target_list.clear();
    }
}

void print_state()
{
    switch (state)
    {
        case State::IDLE:
            ROS_INFO("State = IDLE");
            break;
        
        case State::LOCALIZE:
            ROS_INFO("State = LOCALIZE");
            break;

        case State::ALIGN:
            ROS_INFO("State = ALIGN");
            break;

        case State::EXECUTE_PLAN:
            ROS_INFO("State = EXECUTE_PLAN");
            break;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "local_planner");
    ros::NodeHandle n;
    ros::Rate loop_rate(15);

    ROS_INFO("Starting node.");
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
        ros::console::notifyLoggerLevelsChanged();
    }

    ROS_DEBUG("Waiting for Occupancy Map...");
    map_sub = n.subscribe("grid_map", 1, map_callback);
    while (!map_received)
    {
        
    }
    ROS_DEBUG("Occupancy Map received. Starting localization phase...");

    // Subscribers
    ros::Subscriber sensor_sub = n.subscribe("position", 1, localization_callback);
    // Publishers
    pose_pub = n.advertise<green_fundamentals::Pose>("pose", 1);
    /*bool localizer_available = ros::service::waitForService("localizer_start_localize", ros::Duration(10.0));
    //bool mover_available = ros::service::waitForService("mover_set_idle", ros::Duration(10.0));
    if (!localizer_available || !mover_available)
    {
        ROS_ERROR("A service is not available");
        return 1;
    }
    // Services
    //start_localize_client = n.serviceClient<std_srvs::Empty>("localizer_start_localize");*/
    mover_set_idle_client = n.serviceClient<std_srvs::Empty>("mover_set_idle");
    mover_set_wander_client = n.serviceClient<std_srvs::Empty>("mover_set_wander");
    mover_drive_to_client = n.serviceClient<green_fundamentals::DriveTo>("mover_set_drive_to");
    
    ros::ServiceServer execute_plan_srv = n.advertiseService("execute_plan", set_execute_plan_callback);
    ROS_DEBUG("Advertising execute_plan service");
    
    state = State::IDLE;
    while(ros::ok()) {
        
        if (last_state != state) print_state();
        last_state = state; 

        switch (state)
        {
            case State::IDLE:
                // do nothing
                break;
            
            case State::LOCALIZE:
                localize();
                break;

            case State::ALIGN:
                align();
                break;

            case State::EXECUTE_PLAN:
                execute_plan();
                break;
        }
        
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}