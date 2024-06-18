#include "ros/ros.h"
#include <queue>
#include <deque>
#include <unordered_map>
#include <algorithm>
#include <csignal>

#include "green_fundamentals/Grid.h"
#include "green_fundamentals/GetGlobalPlan.h"
#include "green_fundamentals/GetShortestPath.h"
#include "robot_constants.h"

using Grid_Coords = std::pair<int, int>;
using KeyType = std::pair<Grid_Coords, Grid_Coords>;

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

void print_plan(std::vector<Grid_Coords> plan, int score, Grid_Coords best_endpoint, Grid_Coords current_position)
{
    std::cout << "Plan with Cost: " << score << ": ";
    std::cout << "Start (" << current_position.first << "," << current_position.second << ")";
    for (Grid_Coords coord : plan)
    {
        std::cout << "(" << coord.first << "," << coord.second << ") -> ";
    }

    std::cout << "End (" << best_endpoint.first << "," << best_endpoint.second << ")";

    std::cout << std::endl;
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
            print_plan(path, cost, best_endpoint, {current_cell.row, current_cell.col});
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

bool global_plan_callback(green_fundamentals::GetGlobalPlan::Request  &req, green_fundamentals::GetGlobalPlan::Response &res)
{
    Cell current_cell = cell_grid[req.current_cell.row][req.current_cell.column];
    std::vector<Grid_Coords> best_plan = get_best_plan(current_cell);

    res.global_plan.clear();
    for (Grid_Coords coord : best_plan)
    {
        green_fundamentals::Pose pose;
        pose.row = coord.first;
        pose.column = coord.second;
        res.global_plan.push_back(pose);
    }
    
    return true;
}

bool shortest_path_callback(green_fundamentals::GetShortestPath::Request  &req, green_fundamentals::GetShortestPath::Response &res)
{
    std::vector<Grid_Coords> path = shortest_paths_precomputed[{{req.start.row, req.start.column}, {req.end.row, req.end.column}}];

    res.path.clear();
    for (Grid_Coords coord : path)
    {
        green_fundamentals::Pose pose;
        pose.row = coord.first;
        pose.column = coord.second;
        res.path.push_back(pose);
    }
    
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

    ros::ServiceServer global_plan_srv = n.advertiseService("get_global_plan", global_plan_callback);
    ros::ServiceServer shortest_path_srv = n.advertiseService("get_shortest_path", shortest_path_callback);

    ros::spin();
}