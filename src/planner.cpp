#include "ros/ros.h"
#include "std_srvs/Empty.h"
#include "nav_msgs/OccupancyGrid.h"
#include <deque>
#include <unordered_map>
#include <algorithm>
#include <csignal>
#include <vector>

#include "green_fundamentals/Position.h"
#include "green_fundamentals/Grid.h"
#include "green_fundamentals/Cell.h"
#include "green_fundamentals/Pose.h"
#include "green_fundamentals/DriveTo.h"
#include "green_fundamentals/MoveToPosition.h"
#include "green_fundamentals/GoldRun.h"
#include "green_fundamentals/SetVideo.h"
#include "robot_constants.h"

#define REASONABLE_DISTANCE 0.5
#define LOCALIZATION_POINTS_THRESHOLD 3

using Grid_Coords = std::pair<int, int>;
using KeyType = std::pair<Grid_Coords, Grid_Coords>;

struct pair_hash_int {
    std::size_t operator()(const Grid_Coords& p) const {
        return p.first * 32 + p.second;
    }
};

struct key_hash {
    std::size_t operator()(const KeyType& k) const {
        return pair_hash_int()(k.first) * 64 + pair_hash_int()(k.second);
    }
};

/*
############################################################################
GLOBAL STUFF
############################################################################
*/
ros::ServiceClient mover_drive_to_client, video_player;
/*
###################################
STATE
###################################
*/
enum State {
    INIT,
    LOCALIZE,
    ALIGN,
    IDLE,
    EXECUTE_PLAN,
    NEXT_GOAL,
    GOLD_RUN
};
State state = State::INIT;
/*
###################################
MAP
###################################
*/
struct Cell {
    float x, y;
    int row, col;
    bool wall_left, wall_up, wall_right, wall_down;
};
int grid_rows, grid_cols;
ros::Subscriber map_sub;
bool map_received = false;
std::vector<std::vector<Cell>> cell_grid;
std::vector<Grid_Coords> golds;
std::vector<Grid_Coords> pickups;
std::vector<std::vector<Grid_Coords>> gold_permutations;
std::unordered_map<KeyType, std::vector<Grid_Coords>, key_hash> shortest_paths_precomputed;
std::vector<std::vector<bool>> visited_cells;

void reset_visited_cells() {
    for (int row = 0; row < visited_cells.size(); row++) {
        for (int col = 0; col < visited_cells[row].size(); col++)
        {
            visited_cells[row][col] = false;
        }
    }
}

/*
###################################
GOALS
###################################
*/

std::deque<Grid_Coords> global_plan;
void add_goal_front(int row, int col)
{
    global_plan.push_back({row, col});
}
void add_goal_back(int row, int col)
{
    global_plan.push_front({row, col});
}

/*
###################################
POSITION
###################################
*/

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

Position my_position{0., 0., 0., 0, 0};
bool is_first_position = true;
bool is_localized = false;
int localization_points = 0;

/*
###################################
TARGETS
###################################
*/

struct Target {
    float x, y, theta;
    bool should_rotate = false;
    bool must_be_reached = false;
};

std::deque<Target> local_plan;

void add_target_front(float x, float y, float theta, bool should_rotate, bool must_be_reached)
{
    Target target{x, y, theta, should_rotate, must_be_reached};
    local_plan.push_front(target);
}

void add_target_back(float x, float y, float theta, bool should_rotate, bool must_be_reached)
{
    Target target{x, y, theta, should_rotate, must_be_reached};
    local_plan.push_back(target);
}

/*
###################################
UTILITY FUNCTIONS
###################################
*/

/*
    0 = get_money 
    1 = pickup 
    2 = is_lost 
    3 = is_localized
*/
void set_video(int state)
{
    create_fundamentals::SetVideo srv;

    srv.request.state = state;
    video_player.call(srv);
}

void shutdown(int signum) 
{
    ros::shutdown();
    exit(0);
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

        case State::NEXT_GOAL:
                ROS_INFO("State = NEXT_GOAL");
                break;

        case State::GOLD_RUN:
                ROS_INFO("State = GOLD_RUN");;
                break;
        
        default:
            ROS_INFO("State not knows.");
    }
}

/*
############################################################################
BFS SEARCH
############################################################################
*/

std::vector<Grid_Coords> get_neighbors(const Cell& cell) 
{
    std::vector<Grid_Coords> neighbors;
    int row = cell.row;
    int col = cell.col;

    if (!cell.wall_up && row < grid_rows - 1) {
        neighbors.push_back({row + 1, col});
    }
    if (!cell.wall_down && row > 0) {
        neighbors.push_back({row - 1, col});
    }
    if (!cell.wall_left && col > 0) {
        neighbors.push_back({row, col - 1});
    }
    if (!cell.wall_right && col < grid_cols - 1) {
        neighbors.push_back({row, col + 1});
    }

    return neighbors;
}

std::vector<Grid_Coords> get_shortest_path(const Grid_Coords start, const Grid_Coords end) 
{   
    std::vector<std::vector<bool>> visited(grid_rows, std::vector<bool>(grid_cols, false));
    std::unordered_map<Grid_Coords, Grid_Coords, pair_hash_int> parent;
    parent[{start.first, start.second}] = {-1, -1};

    std::deque<Grid_Coords> q;

    q.push_back({start.first, start.second});
    visited[start.first][start.second] = true;

    //ROS_INFO("Starting Node is (%d, %d)", start.first, start.second);

    while (!q.empty()) {
        auto [currentRow, currentCol] = q.front();
        visited[currentRow][currentCol] = true;
        q.pop_front();

        if (currentRow == end.first && currentCol == end.second) {
            std::vector<Grid_Coords> path;
            for (Grid_Coords at = {end.first, end.second}; at.first != -1; at = parent[at]) {
                path.push_back(at);
            }
            std::reverse(path.begin(), path.end());
            return path;
        }
        
        std::vector<Grid_Coords> neighbors = get_neighbors(cell_grid[currentRow][currentCol]);
        //ROS_INFO("%d Neighbors for cell (%d, %d)", neighbors.size(), currentRow, currentCol);
        for (const auto& neighbor : neighbors) {
            int newRow = neighbor.first;
            int newCol = neighbor.second;
            if (!visited[newRow][newCol]) {
                q.push_back({newRow, newCol});
                parent[{newRow, newCol}] = {currentRow, currentCol};
            }
        }
    }

    return {};
}

/*
############################################################################
COMPUTE GLOBAL PLAN
############################################################################
*/

void compute_gold_permutations()
{
    gold_permutations.clear();
    do {
        gold_permutations.push_back(golds);
    } while (std::next_permutation(golds.begin(), golds.end()));
}

std::vector<Grid_Coords> get_best_plan(const Cell& current_cell)
{
    int best_cost = 100000;
    std::vector<Grid_Coords> best_path;

    for (std::vector<Grid_Coords> path : gold_permutations)
    {
        // Compute cost of path + current_to_first_gold + last_gold_to_shortest_pickup
        int cost = shortest_paths_precomputed[{{current_cell.row, current_cell.col}, path.front()}].size();

        for (int i = 0; i < path.size() - 1; i++)
        {
            cost += shortest_paths_precomputed[{path[i], path[i+1]}].size();
        }
        
        Grid_Coords best_endpoint;
        int best_endpoint_cost = 100000;
        for (Grid_Coords pickup : pickups)
        {   
            int endpoint_cost = shortest_paths_precomputed[{path.back(), pickup}].size();
            if (endpoint_cost < best_endpoint_cost)
            {
                best_endpoint = pickup;
                best_endpoint_cost = endpoint_cost;
            }
        }

        cost += best_endpoint_cost;

        if (cost < best_cost)
        {
            best_cost = cost;
            std::vector<Grid_Coords> temp = path;
            temp.push_back(best_endpoint);
            best_path = temp;
        }
    }

    return best_path;
}

void map_callback(const green_fundamentals::Grid::ConstPtr& msg)
{   
    // ROWS
    grid_rows = msg->rows.size();
    grid_cols = msg->rows[0].cells.size();

    cell_grid.resize(grid_rows);

    for (int row = 0; row < msg->rows.size(); row++)
    {
        std::vector<Cell> column_cells;

        for (int col = 0; col < msg->rows[row].cells.size(); col++)
        {
            Cell new_cell;
            new_cell.x = (float)col * CELL_LENGTH + (CELL_LENGTH / 2);
            new_cell.y = (float)(grid_rows - row - 1) * CELL_LENGTH + (CELL_LENGTH / 2);
            new_cell.row = (grid_rows - row - 1);
            new_cell.col = col;
            new_cell.wall_right = false;
            new_cell.wall_up = false;
            new_cell.wall_left = false;
            new_cell.wall_down = false;
            
            green_fundamentals::Cell current = msg->rows[row].cells[col];
            for (auto wall : current.walls) 
            {
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

        cell_grid[grid_rows - row - 1] = column_cells;
    }

    // GOLDS
    for (int i = 0; i < msg->golds.size(); i++)
    {
        golds.push_back({grid_rows - msg->golds[i].row - 1, msg->golds[i].column});
    }

    // Precompute gold permutations
    compute_gold_permutations();

    // PICKUPS
    for (int i = 0; i < msg->pickups.size(); i++)
    {
        pickups.push_back({grid_rows -  msg->pickups[i].row - 1, msg->pickups[i].column});
    }

    // Precompute shortest paths between all cells
    for (int row1 = 0; row1 < grid_rows; ++row1) 
    {
        for (int col1 = 0; col1 < grid_cols; ++col1) 
        {
            for (int row2 = 0; row2 < grid_rows; ++row2)
            {
                for (int col2 = 0; col2 < grid_cols; ++col2)
                {
                    if (row1 == row2 && col1 == col2) continue;
                    std::vector<Grid_Coords> path = get_shortest_path({row1, col1}, {row2, col2});
                    shortest_paths_precomputed[{{row1, col1}, {row2, col2}}] = path;
                    //std::reverse(path.begin(), path.end()); // For the reverse path
                    //shortest_paths_precomputed[{{end.row, end.col}, {start.row, start.col}}] = path;
                }
            }
        }
    }

    // VISITED CELLS
    std::vector<std::vector<bool>> visited(grid_rows, std::vector<bool>(grid_cols, false));
    visited_cells = visited;

    map_received = true;
    map_sub.shutdown();
}

/*
############################################################################
LOCALIZATION
############################################################################
*/

void localization_callback(const green_fundamentals::Position::ConstPtr& msg)
{   
    float old_x, old_y;
    if (is_first_position)
    {
        old_x = msg->x;
        old_y = msg->y;
        is_first_position = false;
    }
    else 
    {
        old_x = my_position.x;
        old_y = my_position.y;
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
    if (fabs(my_position.x - old_x) > REASONABLE_DISTANCE ||
        fabs(my_position.y - old_y) > REASONABLE_DISTANCE) 
    {
        ROS_INFO("Unreasonable movement");
        reset_visited_cells();
        localization_points = 0;
        local_plan.clear();
    }    
    else if (!visited_cells[my_position.row][my_position.row])
    {
        visited_cells[my_position.row][my_position.row] = true;
        localization_points += 1;
    }
    
    bool was_localized_before = is_localized;
    is_localized = localization_points > LOCALIZATION_POINTS_THRESHOLD;

    if (!is_localized)
    {   
        state = State::LOCALIZE;
        // TODO check
        if (was_localized_before)
        {
            set_video(2);
        }
    }
}

/*
############################################################################
ROBOT MOVER
############################################################################
*/

bool set_local_plan_to_next_goal() 
{

    const Grid_Coords next_goal = global_plan.front();

    std::vector<Grid_Coords> path = shortest_paths_precomputed[{{my_position.row, my_position.col}, next_goal}];

    if (path.empty()) return false;

    local_plan.clear();

    for (int i = 1; i < path.size(); i++) {
        const Cell cell = cell_grid[path[i].first][path[i].second];
        if (cell.row != path[i].first && cell.col != path[i].second)
        {
            ROS_INFO("ERROR in drive_to_cell with cell conversion from first to row and second to col.");
            continue;
        }
        add_target_back(cell.x, cell.y, 0.0, false, i == path.size() - 1);
    }

    return true;
}

void send_next_target_to_mover() 
{
    if (local_plan.empty()) {
        ROS_DEBUG("Cannot set target, because local_plan is empty");
        return;
    }

    green_fundamentals::DriveTo drive_to_msg;

    const Target current_target = local_plan.front();
    
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
    if (local_plan.empty()) {
        return true;
    }

    Target current_target = local_plan.front();

    float epsilon = current_target.must_be_reached ? POS_EPSILON : SOFT_EPSILON;
    float distance = std::sqrt(std::pow(my_position.x - current_target.x, 2) + std::pow(my_position.y - current_target.y, 2));
    ROS_DEBUG("dist: %f, eps %f", distance, epsilon);
    
    bool distance_diff = distance < epsilon;
    bool angle_diff = fabs(my_position.theta - current_target.theta) < THETA_EPSILON;
    
    if (current_target.must_be_reached && current_target.should_rotate) return distance_diff && angle_diff;
    
    return distance_diff;
}
/*
############################################################################
STATE FUNCTIONS
############################################################################
*/

bool move_to_position_callback(green_fundamentals::MoveToPosition::Request  &req, green_fundamentals::MoveToPosition::Response &res)
{
    global_plan.clear();
    add_goal_front(req.row, req.column);
    
    bool success = set_local_plan_to_next_goal();

    if (success)
    {
        state = State::EXECUTE_PLAN;
        ROS_INFO("Added targets to local_plan. Now we have %ld targets.", local_plan.size());
    }
    
    res.success = success;
    return success;
}

bool gold_run_callback(green_fundamentals::GoldRun::Request  &req, green_fundamentals::GoldRun::Response &res)
{
    Cell current_cell;
    current_cell.x = my_position.x;
    current_cell.y = my_position.y;
    current_cell.row = my_position.row;
    current_cell.col = my_position.col;
    std::vector<Grid_Coords> goal_plan = get_best_plan(current_cell);

    for(Grid_Coords goal : goal_plan) {
        add_goal_back(goal.first, goal.second);
        bool success = set_local_plan_to_next_goal();
    }
    
    state = State::EXECUTE_PLAN;
    ROS_INFO("Added targets to local_plan. Now we have %ld targets.", local_plan.size());
    
    return true;
}

void execute_local_plan()
{
    if (local_plan.empty())  
    {
        state = State::NEXT_GOAL;
        return;
    }
    
    if (current_target_reached())
    {   
        local_plan.pop_front();
    }
    
    send_next_target_to_mover();
}

void get_next_goal()
{
    if (global_plan.size() > 1)
    {
        global_plan.pop_front();
        set_local_plan_to_next_goal();
        state = State::EXECUTE_PLAN;
    }
    else if (global_plan.size() == 1)
    {
        global_plan.pop_front();
        state = State::IDLE;
    }
    else 
    {
        state = State::IDLE;
    }
}

void localize()
{   
    if (is_localized)
    {
        state = State::ALIGN;
        local_plan.clear();
        global_plan.clear();
        // TODO check
        set_video(3);
        return;
    }

    // Not localized

    if (local_plan.empty())
    {
        Cell unvisited_cell;
        bool found_unvisited_cell = false;

        while (!found_unvisited_cell) {
            int rand_row = rand() % grid_rows;
            int rand_col = rand() % grid_cols;
            if (!visited_cells[rand_row][rand_col]) {
                global_plan.clear();
                add_goal_front(rand_row, rand_col);
                found_unvisited_cell = set_local_plan_to_next_goal();
            }
        }

        return;
    }

    // not localized && local_plan not empty

    if (current_target_reached())
    {   
        local_plan.pop_front();
    }

    send_next_target_to_mover();
}

void align()
{
    const Cell cell = cell_grid[my_position.row][my_position.col];
    add_target_front(cell.x, cell.y, M_PI/2, true, true);
    send_next_target_to_mover();
    
    state = State::IDLE;
    local_plan.clear();
}

/*
###################################
MAIN LOOP
###################################
*/
int main(int argc, char **argv)
{
    ros::init(argc, argv, "planner");
    ros::NodeHandle n;
    ros::Rate loop_rate(30);

    signal(SIGINT, shutdown);

    ROS_INFO("Starting node.");
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info)) {
        ros::console::notifyLoggerLevelsChanged();
    }

    ROS_INFO("Waiting for Occupancy Map...");
    map_sub = n.subscribe("grid_map", 1, map_callback);
    while (!map_received)
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    ROS_INFO("Map received and processed.");

    ros::Subscriber sensor_sub = n.subscribe("position", 1, localization_callback);
    mover_drive_to_client = n.serviceClient<green_fundamentals::DriveTo>("mover_set_drive_to");
    video_player = n.serviceClient<green_fundamentals::SetVideo>("set_video");
    
    ros::ServiceServer move_to_position_srv = n.advertiseService("move_to_position", move_to_position_callback);
    ros::ServiceServer gold_run_srv = n.advertiseService("gold_run", gold_run_callback);
    
    state = State::IDLE;
    State last_state = state;
    while(ros::ok()) {
        ros::spinOnce();
        
        if (last_state != state) print_state();
        last_state = state; 

        if (is_first_position) continue;

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
                execute_local_plan();
                break;
            
            case State::NEXT_GOAL:
                get_next_goal();
                break;

            case State::GOLD_RUN:
                get_next_goal();
                break;

            default:
                ROS_INFO("State not known.");
        }
        
        loop_rate.sleep();
    }

    return 0;
}