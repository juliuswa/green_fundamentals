#include "ros/ros.h"
#include "std_srvs/Empty.h"
#include "nav_msgs/OccupancyGrid.h"
#include <deque>

#include "green_fundamentals/Position.h"
#include "green_fundamentals/Pose.h"
#include "green_fundamentals/DriveTo.h"
#include "green_fundamentals/ExecutePlan.h"
#include "robot_constants.h"

#define REASONABLE_DISTANCE 0.3
#define LOCALIZATION_POINTS_THRESHOLD 3

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
int localization_points = 0;

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
std::vector<std::vector<int8_t>> map_data;
int map_height, map_width, grid_rows, grid_cols, wall_length_pixels, corner_pixels, wall_length;
bool map_received = false;
std::vector<std::vector<Cell>> cell_centers;

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

void map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{   
    ROS_DEBUG("Inside map_callback...");
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

    map_received = true;
    map_sub.shutdown();
    ROS_DEBUG("Map parsed and unsubscribed from map topic: map_height=%d map_width=%d", map_height, map_width);

    int wall_length_pixels, corner_pixels;
    float pixel_size;

    ros::param::get("grid_num_rows", grid_rows);
    ros::param::get("grid_num_cols", grid_cols);
    ros::param::get("wall_length_pixels", wall_length_pixels);
    ros::param::get("corner_pixels", corner_pixels);
    ros::param::get("pixel_size", pixel_size);

    ROS_DEBUG("Received parameters from server: grid_num_rows=%d grid_num_cols=%d wall_length_pixels=%d corner_pixels=%d pixel_size=%f",
              grid_rows, grid_cols, wall_length_pixels, corner_pixels, pixel_size);

    wall_length = (wall_length_pixels + corner_pixels) * pixel_size;
    ROS_DEBUG("Computed wall_length=%f", wall_length);

    ROS_DEBUG("Computing cell centers...");
    // compute the cell centers
    cell_centers.resize(grid_rows);
    for (int i = 0; i < grid_rows; ++i) 
    {
        cell_centers[i].resize(grid_cols);
    }

    for (int row = 0; row < grid_rows; row++)
    {
        for (int col = 0; col < grid_cols; col++)
        {
            Cell cell;

            int center_y_pixel = map_height - row * (corner_pixels + wall_length_pixels) - corner_pixels - wall_length_pixels / 2;
            int center_x_pixel = col * (corner_pixels + wall_length_pixels) + corner_pixels + wall_length_pixels / 2;

            // add cell information in a Cell struct so that we can check when executing plan if we will run into a wall.
            for (int i = 1; i <= wall_length_pixels; i++)
            {
                cell.wall_right = map_data[std::min(center_x_pixel + i, map_width)][center_y_pixel] != 0 ? true : cell.wall_right;
                cell.wall_left = map_data[std::max(center_x_pixel - i, 0)][center_y_pixel] != 0 ? true : cell.wall_left;
                cell.wall_up = map_data[center_x_pixel][std::min(center_y_pixel + i, map_width)] != 0 ? true : cell.wall_up;
                cell.wall_down = map_data[center_x_pixel][std::max(center_y_pixel - i, 0)] != 0 ? true : cell.wall_down;
            }

            cell.y = center_y_pixel * pixel_size;
            cell.x = center_x_pixel * pixel_size;
            cell_centers[row][col] = cell;
            ROS_DEBUG("Cell Centers row=%d col=%d: x=%f y=%f left=%d up=%d right=%d down=%d", 
            row, col, cell.x, cell.y, cell.wall_left, cell.wall_up, cell.wall_right, cell.wall_down);
        }
    }
    ROS_DEBUG("End of map_callback");
}

void localization_callback(const green_fundamentals::Position::ConstPtr& msg)
{   
    if (is_first_position)
    {
        old_position.x = msg->x;
        old_position.y = msg->y;
        old_position.theta = msg->theta;
        old_position.row = grid_rows - floor(my_position.y / wall_length);
        old_position.col = floor(my_position.x / wall_length);
        is_first_position = false;
    }
    else
    {
        old_position.x = my_position.x;
        old_position.y = my_position.y;
        old_position.theta = my_position.theta;
        old_position.row = my_position.row;
        old_position.col = my_position.col;
        //old_position.orientation = my_position.orientation;
    }

    my_position.x = msg->x;
    my_position.y = msg->y;
    my_position.theta = msg->theta;
    //my_position.converged = msg->converged;
    my_position.row = grid_rows - floor(my_position.y / wall_length);
    my_position.col = floor(my_position.x / wall_length);

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
    if (fabs(my_position.x - old_position.x) + fabs(my_position.y - old_position.y) < REASONABLE_DISTANCE) 
    {
        localization_points = 0;
    }
    
    if (old_position.row != my_position.row || old_position.col != my_position.col)
    {
        // check if new cell is reachable from old cell
        Cell old_cell = cell_centers[old_position.row][old_position.col];

        if (abs(old_position.row - my_position.row) + abs(old_position.col - my_position.col) > 1)
        {
            localization_points = 0;
        }
        else if (old_position.row > my_position.row && old_cell.wall_left)
        {
            localization_points = 0;
        }
        else if (old_position.row < my_position.row && old_cell.wall_right)
        {
            localization_points = 0;
        }
        else if (old_position.col > my_position.col && old_cell.wall_down)
        {
            localization_points = 0;
        }
        else if (old_position.col < my_position.col && old_cell.wall_up)
        {
            localization_points = 0;
        }
        else {
            localization_points += 1;
        }
    }
    
    is_localized = localization_points > LOCALIZATION_POINTS_THRESHOLD;

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
    Cell cell = cell_centers[my_position.row][my_position.col];
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
            move_possible = !cell_centers[row][col].wall_right;
            col++;
            break;
        case  1: // UP
            move_possible = !cell_centers[row][col].wall_up;
            row++;
            break;
        case  2: // LEFT
            move_possible = !cell_centers[row][col].wall_left;
            col--;
            break;
        case  3: // DOWN
            move_possible = !cell_centers[row][col].wall_down;
            row--;
            break;
        }

        if (!move_possible || row > grid_rows || row < 0 || col > grid_cols || col < 0)
        {
            target_list.clear();
            res.success = false;
            state = State::IDLE;
            ROS_DEBUG("Plan cannot be fulfilled, because we would move out of the maze or into a wall.");
            ROS_INFO("State = IDLE");
            return false;
        }

        Cell center = cell_centers[row][col];
        new_target.x = center.x;
        new_target.y = center.y;
        new_target.must_be_reached = i == req.plan.size()-1 ? true : false;
        target_list.push_back(new_target);
    }

    state = State::EXECUTE_PLAN;
    ROS_DEBUG("Added targets to target_list. Now we have %d targets.", target_list.size());
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
    if (is_localized)
    {
        state = State::ALIGN;
        target_list.clear();
        return;
    }

    // Go to neigboring cell
    if (target_list.empty())
    {
        Cell current_cell = cell_centers[my_position.row][my_position.col];
        Cell neighbor_cell;
        bool found_neighbor = false;

        while (!found_neighbor) {
            int rand_wall = rand() % 4;
            switch (rand_wall)
            {
                case 0:
                    if (!current_cell.wall_right)  {
                        neighbor_cell = cell_centers[my_position.row][my_position.col + 1];
                        found_neighbor = true;
                    }
                    break;
                case 1:
                    if (!current_cell.wall_up)  {
                        neighbor_cell = cell_centers[my_position.row + 1][my_position.col];
                        found_neighbor = true;
                    }
                    break;
                case 2:
                    if (!current_cell.wall_left)  {
                        neighbor_cell = cell_centers[my_position.row][my_position.col - 1];
                        found_neighbor = true;
                    }
                    break;
                case 3:
                    if (!current_cell.wall_down)  {
                        neighbor_cell = cell_centers[my_position.row - 1][my_position.col];
                        found_neighbor = true;
                    }
                    break;
            }
        }

        add_target_front(neighbor_cell.x, neighbor_cell.y, M_PI/2, false, false);
        set_target();
    }
}

void align()
{
    if (target_list.empty())
    {
        std::pair<float, float> target = get_current_cell_center(); // x, y
        ROS_DEBUG("Current cell is (%d, %d)", my_position.row, my_position.col);
        ROS_DEBUG("Cell Center is (%d, %d)", target.first, target.second);
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
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info)) {
        ros::console::notifyLoggerLevelsChanged();
    }

    map_sub = n.subscribe("map", 1, map_callback);
    while (!map_received)
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    // Subscribers
    ros::Subscriber sensor_sub = n.subscribe("position", 1, localization_callback);
    // Publishers
    pose_pub = n.advertise<green_fundamentals::Pose>("pose", 1);
    // Services
    mover_set_idle_client = n.serviceClient<std_srvs::Empty>("mover_set_idle");
    mover_set_wander_client = n.serviceClient<std_srvs::Empty>("mover_set_wander");
    mover_drive_to_client = n.serviceClient<green_fundamentals::DriveTo>("mover_set_drive_to");
    
    ros::ServiceServer execute_plan_srv = n.advertiseService("execute_plan", set_execute_plan_callback);
    
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