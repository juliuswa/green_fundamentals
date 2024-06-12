#include "ros/ros.h"
#include <queue>
#include <unordered_map>
#include <algorithm>
#include <csignal>

#include "green_fundamentals/Grid.h"
#include "green_fundamentals/GetGlobalPlan.h"
#include "robot_constants.h"

using Grid_Coords = std::pair<int, int>;
using KeyType = std::pair<Grid_Coords, Grid_Coords>;

struct pair_hash_int {
    std::size_t operator()(const Grid_Coords& p) const {
        return std::hash<int>()(p.first) ^ std::hash<int>()(p.second);
    }
};

// Hash function for the key type
struct key_hash {
    std::size_t operator()(const KeyType& k) const {
        return pair_hash_int()(k.first) ^ pair_hash_int()(k.second);
    }
};

struct Cell {
    float x, y;
    int row, col;
    bool wall_left, wall_up, wall_right, wall_down;
};

std::vector<std::vector<Cell>> cell_grid;
std::vector<Grid_Coords> golds;
std::vector<Grid_Coords> pickups;
std::vector<std::vector<Grid_Coords>> gold_permutations;
std::unordered_map<KeyType, std::vector<Grid_Coords>, key_hash> shortest_paths_precomputed;

ros::Subscriber map_sub;
bool map_received = false;
int grid_rows, grid_cols;

std::vector<Grid_Coords> get_neighbors(const Cell& cell) {
    std::vector<Grid_Coords> neighbors;
    int row = cell.row;
    int col = cell.col;

    if (!cell.wall_up && row > 0) {
        neighbors.push_back({row - 1, col});
    }
    if (!cell.wall_down && row < cell_grid.size() - 1) {
        neighbors.push_back({row + 1, col});
    }
    if (!cell.wall_left && col > 0) {
        neighbors.push_back({row, col - 1});
    }
    if (!cell.wall_right && col < cell_grid[0].size() - 1) {
        neighbors.push_back({row, col + 1});
    }

    return neighbors;
}

std::vector<Grid_Coords> get_shortest_path(const Cell& start, const Cell& end) 
{   
    std::vector<std::vector<bool>> visited(grid_rows, std::vector<bool>(grid_cols, false));
    std::unordered_map<Grid_Coords, Grid_Coords, pair_hash_int> parent;
    parent[{start.row, start.col}] = {-1, -1};

    std::queue<Grid_Coords> q;

    q.push({start.row, start.col});
    visited[start.row][start.col] = true;

    while (!q.empty()) {
        auto [currentRow, currentCol] = q.front();
        q.pop();

        if (currentRow == end.row && currentCol == end.col) {
            std::vector<Grid_Coords> path;
            for (Grid_Coords at = {end.row, end.col}; at.first != -1; at = parent[at]) {
                path.push_back(at);
            }
            std::reverse(path.begin(), path.end());
            return path;
        }
        
        for (const auto& neighbor : get_neighbors(cell_grid[currentRow][currentCol])) {
            int newRow = neighbor.first;
            int newCol = neighbor.second;
            if (!visited[newRow][newCol]) {
                q.push({newRow, newCol});
                visited[newRow][newCol] = true;
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
        //[{{start.row, start.col}, {end.row, end.col}}]
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
            best_path = path;
        }
    }

    return best_path;
}

void map_callback(const green_fundamentals::Grid::ConstPtr& msg)
{   
    // ROWS
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
            new_cell.row = row;
            new_cell.col = col;
            new_cell.wall_right = false;
            new_cell.wall_up = false;
            new_cell.wall_left = false;
            new_cell.wall_down = false;
            
            green_fundamentals::Cell current = msg->rows[msg->rows.size() - (row + 1)].cells[col];
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

        cell_grid.push_back(column_cells);
    }

    // GOLDS
    for (int i = 0; i < msg->golds.size(); i++)
    {
        golds.push_back({msg->golds[i].row, msg->golds[i].column});
    }

    // Precompute gold permutations
    compute_gold_permutations();

    // PICKUPS
    for (int i = 0; i < msg->pickups.size(); i++)
    {
        pickups.push_back({msg->pickups[i].row, msg->pickups[i].column});
    }

    // Precompute shortest paths between all cells
    for (int row1 = 0; row1 < grid_rows; ++row1) 
    {
        for (int col1 = 0; col1 < grid_cols; ++col1) 
        {
            for (int row2 = row1; row2 < grid_rows; ++row2)
            {
                for (int col2 = (row2 == row1 ? col1 + 1 : 0); col2 < grid_cols; ++col2)
                {
                    const Cell& start = cell_grid[row1][col1];
                    const Cell& end = cell_grid[row2][col2];
                    std::vector<Grid_Coords> path = get_shortest_path(start, end);
                    shortest_paths_precomputed[{{start.row, start.col}, {end.row, end.col}}] = path;
                    std::reverse(path.begin(), path.end()); // For the reverse path
                    shortest_paths_precomputed[{{end.row, end.col}, {start.row, start.col}}] = path;
                }
            }
        }
    }

    map_received = true;
    map_sub.shutdown();
}

bool get_global_plan(green_fundamentals::GetGlobalPlan::Request  &req, green_fundamentals::GetGlobalPlan::Response &res)
{
    Cell current_cell = cell_grid[req.current_cell.row][req.current_cell.column];
    std::vector<Grid_Coords> best_plan = get_best_plan(current_cell);

    std::cout << "Best plan ist:" << " ";
    res.global_plan.clear();
    for (Grid_Coords coord : best_plan)
    {
        green_fundamentals::Pose pose;
        pose.row = coord.first;
        pose.column = coord.second;
        res.global_plan.push_back(pose);
        std::cout << "(" << coord.first << "," << coord.second << ") ";
    }
    std::cout << std::endl;
    
    return true;
}

void shutdown(int signum) 
{
    ros::shutdown();
}

int main(int argc, char **argv)
{
    signal(SIGINT, shutdown);
    ros::init(argc, argv, "global_planner");
    ros::NodeHandle n;
    ros::Rate loop_rate(15);

    ROS_INFO("Waiting for Grid Map...");
    map_sub = n.subscribe("grid_map", 1, map_callback);
    while (!map_received)
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    ROS_INFO("Map received and processed.");

    ros::ServiceServer execute_plan_srv = n.advertiseService("get_global_plan", get_global_plan);

    ros::spin();
}