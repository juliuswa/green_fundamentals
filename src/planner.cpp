#include "ros/ros.h"
#include "std_srvs/Empty.h"
#include "nav_msgs/OccupancyGrid.h"
#include <deque>
#include <unordered_map>
#include <algorithm>
#include <csignal>

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

using Grid_Coords = std::pair<int, int>;
using KeyType = std::pair<Grid_Coords, Grid_Coords>;

/*
############################################################################
GLOBAL STUFF
############################################################################
*/

// ROS
ros::ServiceClient mover_drive_to_client, play_song;

// Map
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

/*
############################################################################
GLOBAL PLANNER
############################################################################
*/

struct pair_hash_int {
    std::size_t operator()(const Grid_Coords& p) const {
        return p.first * 32 + p.second;
    }
};

// Hash function for the key type
struct key_hash {
    std::size_t operator()(const KeyType& k) const {
        return pair_hash_int()(k.first) * 64 + pair_hash_int()(k.second);
    }
};

// BFS Search
std::vector<Grid_Coords> get_neighbors(const Cell& cell) {
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

    map_received = true;
    map_sub.shutdown();
}

/*
############################################################################
LOCAL PLANNER
############################################################################
*/

// STATE
enum State {
    INIT,
    LOCALIZE,
    ALIGN,
    IDLE,
    EXECUTE_PLAN,
    MOVE_TO_GOAL,
    TEMPLE_RUN_SIMULATION
};
State state = State::INIT;

// LOCALIZATION
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
bool visited_cells[MAP_WIDTH * MAP_HEIGHT];
int localization_points = 0;

// DRIVING
struct Target {
    float x, y, theta;
    bool should_rotate = false;
    bool must_be_reached = false;
    bool sent = false;
};
std::deque<Target> target_list;

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

bool drive_to_cell(int col, int row) {
    std::vector<Grid_Coords> path = shortest_paths_precomputed[{{my_position.row, my_position.col}, {row, col}}];

    for (int i = 0; i < path.size(); i++) {
        Cell cell = cell_grid[path[i].first][path[i].second];
        if (cell.row != path[i].first && cell.col != path[i].second)
        {
            ROS_INFO("ERROR in drive_to_cell with cell conversion from first to row and second to col.");
            continue;
        }
        add_target_back(cell.x, cell.y, 0.0, false, i == path.size() - 1);
    }

    return !path.empty();
}

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
        target_list.clear();
    }    
    else if (!visited_cells[my_position.col + my_position.row * MAP_WIDTH])
    {
        visited_cells[my_position.col + my_position.row * MAP_WIDTH] = true;
        localization_points += 1;
        ROS_DEBUG("new visited cell: (%d, %d), points = %d", my_position.col, my_position.row, localization_points);
    }
    
    bool was_localized_before = is_localized;
    is_localized = localization_points > LOCALIZATION_POINTS_THRESHOLD;

    if (!is_localized)
    {   
        state = State::LOCALIZE;
        // TODO check
        if (was_localized_before)
        {
            play_song_not_localized();
        }
    }
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

    float epsilon = current_target.must_be_reached ? POS_EPSILON : SOFT_EPSILON;
    float distance = std::sqrt(std::pow(my_position.x - current_target.x, 2) + std::pow(my_position.y - current_target.y, 2));
    ROS_DEBUG("dist: %f, eps %f", distance, epsilon);
    
    bool distance_diff = distance < epsilon;
    bool angle_diff = fabs(my_position.theta - current_target.theta) < THETA_EPSILON;
    
    if (current_target.must_be_reached && current_target.should_rotate) return distance_diff && angle_diff;
    
    return distance_diff;
}

bool set_execute_plan_callback(green_fundamentals::ExecutePlan::Request  &req, green_fundamentals::ExecutePlan::Response &res)
{
    target_list.clear();

    int row = my_position.row;
    int col = my_position.col;

    for (int i = 0; i < req.plan.size(); i++)
    {
        bool move_possible = true;

        switch (req.plan[i])
        {
        case  0: // RIGHT
            move_possible = !cell_grid[row][col].wall_right;
            col++;
            break;
        case  1: // UP
            move_possible = !cell_grid[row][col].wall_up;
            row++;
            break;
        case  2: // LEFT
            move_possible = !cell_grid[row][col].wall_left;
            col--;
            break;
        case  3: // DOWN
            move_possible = !cell_grid[row][col].wall_down;
            row--;
            break;
        }

        if (!move_possible || row > cell_grid.size() || row < 0 || col > cell_grid[0].size() || col < 0)
        {
            target_list.clear();
            res.success = false;
            state = State::IDLE;
            ROS_DEBUG("Plan cannot be fulfilled, because we would move out of the maze or into a wall.");
            ROS_INFO("State = IDLE");
            return false;
        }

        Cell center = cell_grid[row][col];
        add_target_back(center.x, center.y, 0.0, false, i == req.plan.size()-1);
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
        set_target();
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
    Cell cell = cell_grid[my_position.row][my_position.col];
    add_target_front(cell.x, cell.y, M_PI/2, true, true);
    set_target();
    
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

void shutdown(int signum) 
{
    ros::shutdown();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "planner");
    ros::NodeHandle n;
    ros::Rate loop_rate(30);

    signal(SIGINT, shutdown);

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

    ros::Subscriber sensor_sub = n.subscribe("position", 1, localization_callback);
    mover_drive_to_client = n.serviceClient<green_fundamentals::DriveTo>("mover_set_drive_to");
    play_song = n.serviceClient<create_fundamentals::PlaySong>("play_song");
    
    ros::ServiceServer assignment_srv = n.advertiseService("move_to_position", set_execute_plan_callback);
    
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
                execute_plan();
                break;
            
            case State::MOVE_TO_GOAL:
                move_to_goal();
                break;
            
            case State::TEMPLE_RUN_SIMULATION:
                temple_run();
                break;
        }
        
        loop_rate.sleep();
    }

    return 0;
}