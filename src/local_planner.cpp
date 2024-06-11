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
#include "create_fundamentals/PlaySong.h"
#include "create_fundamentals/StoreSong.h"
#include "robot_constants.h"

#define REASONABLE_DISTANCE 0.5
#define LOCALIZATION_POINTS_THRESHOLD 5

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
bool visited_cells[MAP_WIDTH * MAP_HEIGHT];
int localization_points = 0;

ros::Time last_sent_command;

struct Target {
    float x, y, theta;
    bool should_rotate = false;
    bool must_be_reached = false;
    bool sent = false;
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
int grid_rows, grid_cols;
ros::Subscriber map_sub;
bool map_received = false;
std::vector<std::vector<Cell>> cell_info;

// Publishers
ros::Publisher pose_pub;

// Clients
ros::ServiceClient start_localize_client, mover_set_idle_client, mover_set_wander_client, mover_drive_to_client, store_song, play_song;

void reset_visited_cells() {
    for (int i = 0; i < sizeof(visited_cells) / sizeof(bool); i++) {
        visited_cells[i] = false;
    }
}
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

void play_note(int i) {
    create_fundamentals::StoreSong store_srv;
    store_srv.request.number = 0;
    store_srv.request.song = {60,40};

    if(play_song.call(store_srv))
    {
        create_fundamentals::PlaySong play_srv;
        play_srv.request.number = 0;
        play_song.call(play_srv);
    };
}

// TODO check if number is correct
void play_song_localized()
{
    create_fundamentals::PlaySong srv;

    srv.request.number = 1;
    play_song.call(srv);
}

void play_song_not_localized()
{
    create_fundamentals::PlaySong srv;

    srv.request.number = 1;
    play_song.call(srv);
}

std::vector<std::pair<int, int>> get_neighbor_path(int col, int row, std::vector<std::pair<int, int>> current_path) {
    ROS_DEBUG("get_neighbor_path current path length: %ld", current_path.size());
    std::pair<int, int> neighbor_cell {col, row};            
    std::vector<std::pair<int, int>> neighbor_path = current_path;

    neighbor_path.push_back(neighbor_cell);

    return neighbor_path;
}

void drive_to_cell(int col, int row) {
    std::deque<std::vector<std::pair<int, int>>> bfs_deque;
    int num_cells = cell_info.size() * cell_info[0].size();
    bool checked[num_cells];

    for (int i = 0; i < num_cells; i++) {
        checked[i] = false;
    }

    std::vector<std::pair<int, int>> first_path;
    std::pair<int, int> first_cell {my_position.col, my_position.row};
    first_path.push_back(first_cell);
    bfs_deque.push_back(first_path);

    std::vector<std::pair<int, int>> final_cell_path;

    ROS_DEBUG("searching path to: (%d, %d)", col, row);

    while (bfs_deque.size() != 0) {
        std::vector<std::pair<int, int>> current_path = bfs_deque.front();        
        bfs_deque.pop_front();

        std::pair<int, int> current_cell = current_path.back();

        ROS_DEBUG("current_cell: (%d, %d)", current_cell.first, current_cell.second);

        if (checked[current_cell.first + current_cell.second * MAP_WIDTH]) {
            continue;
        }
        
        checked[current_cell.first + current_cell.second * MAP_WIDTH] = true;

        if (current_cell.first == col && current_cell.second == row) {
            ROS_INFO("found path");

            for (int i = 0; i < current_path.size(); i ++) {
                ROS_INFO("(%d, %d)", current_path[i].first,  current_path[i].second);
            }

            final_cell_path = current_path;

            bfs_deque.clear();
            continue;
        }

        int cell_x = current_cell.first;
        int cell_y = current_cell.second;

        if (cell_info[cell_y][cell_x].wall_right == false) {            
            ROS_DEBUG("neighbor right");
            bfs_deque.push_back(get_neighbor_path(cell_x + 1, cell_y, current_path));
        }
        if (cell_info[cell_y][cell_x].wall_up == false) {            
            ROS_DEBUG("neighbor up");
            bfs_deque.push_back(get_neighbor_path(cell_x, cell_y + 1, current_path));
        }
        if (cell_info[cell_y][cell_x].wall_left == false) {   
            ROS_DEBUG("neighbor left");         
            bfs_deque.push_back(get_neighbor_path(cell_x - 1, cell_y, current_path));
        }
        if (cell_info[cell_y][cell_x].wall_down == false) {    
            ROS_DEBUG("neighbor down");        
            bfs_deque.push_back(get_neighbor_path(cell_x, cell_y - 1, current_path));
        }
    }

    ROS_DEBUG("found path of length %ld", final_cell_path.size());

    for (int i = 0; i < final_cell_path.size(); i++) {
        Cell cell = cell_info[final_cell_path[i].second][final_cell_path[i].first];
        Target target;
        target.x = cell.x;
        target.y = cell.y;
        target.theta = 0.0;
        target.should_rotate = false;
        target.must_be_reached = i == final_cell_path.size() - 1;

        ROS_DEBUG("adding target: (%f, %f), mbr %d", target.x, target.y, target.must_be_reached);

        target_list.push_back(target);
    }
}

void map_callback(const green_fundamentals::Grid::ConstPtr& msg)
{   
    ROS_DEBUG("Inside map_callback...");

    grid_rows = msg->rows.size();
    grid_cols = msg->rows[0].cells.size();

    for (int row = 0; row < msg->rows.size(); row++)
    {
        std::vector<Cell> column_cells;

        for (int col = 0; col < msg->rows[row].cells.size(); col++)
        {
            Cell new_cell;
            new_cell.x = (float)col * CELL_LENGTH + (CELL_LENGTH / 2);
            new_cell.y = (float)row * CELL_LENGTH + (CELL_LENGTH / 2);
            new_cell.wall_right = false;
            new_cell.wall_up = false;
            new_cell.wall_left = false;
            new_cell.wall_down = false;
            
            green_fundamentals::Cell current = msg->rows[msg->rows.size() - (row + 1)].cells[col];
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

            ROS_DEBUG("Cell (%d, %d): center: (%f, %f), walls: (%d, %d, %d, %d)",
                col, row, 
                new_cell.x, new_cell.y,
                new_cell.wall_right, new_cell.wall_up, new_cell.wall_left, new_cell.wall_down);

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
        my_position.x = msg->x;
        my_position.y = msg->y;
        my_position.theta = msg->theta;
        my_position.row = floor(my_position.y / CELL_LENGTH);
        my_position.col = floor(my_position.x / CELL_LENGTH);
        is_first_position = false;
    }
     
    old_position.x = my_position.x;
    old_position.y = my_position.y;
    old_position.theta = my_position.theta;
    old_position.row = my_position.row;
    old_position.col = my_position.col;

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
    if (fabs(my_position.x - old_position.x) > REASONABLE_DISTANCE ||
        fabs(my_position.y - old_position.y) > REASONABLE_DISTANCE) 
    {
        ROS_INFO("Unreasonable movement");
        reset_visited_cells();
        localization_points = 0;
        target_list.clear();
    }    
    else if (!visited_cells[my_position.col + my_position.row * MAP_WIDTH])
    {
        visited_cells[my_position.col + my_position.row * MAP_WIDTH] = true;
        localization_points += 1;
        play_note(0);
        ROS_DEBUG("new visited cell: (%d, %d), points = %d", my_position.col, my_position.row, localization_points);
    }
    
    bool was_localized_before = is_localized;
    is_localized = localization_points > LOCALIZATION_POINTS_THRESHOLD;

    if (is_localized)
    {   
        green_fundamentals::Pose pose;

        pose.row = grid_rows - 1 - my_position.row; // TODO check
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
        // TODO check
        if (was_localized_before)
        {
            play_song_not_localized();
        }
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
    current_target.sent = true;

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

    res.success = true;
    return true;
}

void execute_plan()
{
    if (target_list.empty())  {
        state = State::IDLE;
        return;
    }
    else if (current_target_reached())
    {
        target_list.pop_front();
        ROS_INFO("Next Target reached.");
    }
    else if (!target_list[0].sent) {
        set_target();
    }
}

void localize()
{   
    if (is_localized)
    {
        state = State::ALIGN;
        target_list.clear();
        // TODO check
        play_song_localized();
        return;
    }

    if (current_target_reached()) 
    {
        ROS_DEBUG("Target reached, %ld targets remaining.",  target_list.size());

        if (!target_list.empty())
        {
            target_list.pop_front();    
        }
    }

    if (!target_list.empty())
    {
        set_target();        
    }
    else {
        ROS_DEBUG("finding new unvisited cell ...");

        Cell unvisited_cell;
        bool found_unvisied_cell = false;

        while (!found_unvisied_cell) {
            int rand_cell = rand() % (MAP_WIDTH * MAP_HEIGHT);
            if (!visited_cells[rand_cell]) {
                int x = rand_cell % MAP_WIDTH;
                int y = floor(rand_cell / MAP_HEIGHT);
                
                ROS_DEBUG("new unvisited cell: (%d, %d)", x, y);

                drive_to_cell(x, y);
                found_unvisied_cell = true;
            }
        }
    }  
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
    
    state = State::IDLE;
    target_list.clear();
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
        ros::spinOnce();
        loop_rate.sleep();
    }
    ROS_DEBUG("Map received and processed.");

    // Subscribers
    ros::Subscriber sensor_sub = n.subscribe("position", 1, localization_callback);
    // Publishers
    pose_pub = n.advertise<green_fundamentals::Pose>("pose", 1);
    /*bool localzer_available = ros::service::waitForService("localizer_start_localize", ros::Duration(10.0));
    //bool mover_available = ros::service::waitForService("miover_set_idle", ros::Duration(10.0));
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
    store_song = n.serviceClient<create_fundamentals::StoreSong>("store_song");
    play_song = n.serviceClient<create_fundamentals::PlaySong>("play_song");
    
    ros::ServiceServer execute_plan_srv = n.advertiseService("execute_plan", set_execute_plan_callback);
    ROS_DEBUG("Advertising execute_plan service");
    
    last_sent_command = ros::Time::now();
    state = State::IDLE;
    while(ros::ok()) {
        ros::spinOnce();
        
        if (last_state != state) print_state();
        last_state = state; 

        if (is_first_position) continue;

        // ROS_DEBUG("current pos: (%f, %f), th: %f", my_position.x, my_position.y, my_position.theta);

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
        
        loop_rate.sleep();
    }

    return 0;
}