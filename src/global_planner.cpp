#include "ros/ros.h"
#include <queue>
#include <unordered_map>
#include <algorithm>

#include "green_fundamentals/Grid.h"
#include "robot_constants.h"

struct Cell {
    float x, y;
    int row, col;
    bool wall_left, wall_up, wall_right, wall_down;
};
std::vector<std::vector<Cell>> cell_grid;

ros::Subscriber map_sub;
bool map_received = false;
int grid_rows, grid_cols;
void map_callback(const green_fundamentals::Grid::ConstPtr& msg)
{   
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

        cell_grid.push_back(column_cells);
    }

    map_received = true;
    map_sub.shutdown();
}

struct pair_hash_int {
    template <class T1, class T2>
    std::size_t operator()(const std::pair<T1, T2>& p) const {
        auto h1 = std::hash<T1>{}(p.first);
        auto h2 = std::hash<T2>{}(p.second);
        return h1 ^ (h2 << 1); // Combine the hashes
    }
};

struct pair_hash_cell {
    std::size_t operator()(const std::pair<Cell, Cell>& p) const {
        auto hash1 = std::hash<int>{}(p.first.row) ^ std::hash<int>{}(p.first.col << 1);
        auto hash2 = std::hash<int>{}(p.second.row) ^ std::hash<int>{}(p.second.col << 1);
        return hash1 ^ hash2;
    }
};

std::vector<std::vector<Cell>> get_all_permutations(std::vector<Cell> cells) {
    std::vector<std::vector<Cell>> permutations;

    do {
        permutations.push_back(cells);
    } while (std::next_permutation(cells.begin(), cells.end()));

    return permutations;
}

std::vector<std::pair<int, int>> get_neighbors(const Cell& cell) {
    std::vector<std::pair<int, int>> neighbors;
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

std::vector<Cell> get_shortest_path(const Cell& start, const Cell& end) 
{
    int rows = cell_grid.size();
    int cols = cell_grid[0].size();
    
    std::vector<std::vector<bool>> visited(rows, std::vector<bool>(cols, false));
    std::unordered_map<std::pair<int, int>, std::pair<int, int>, pair_hash_int> parent;
    parent[{start.row, start.col}] = {-1, -1};

    std::queue<std::pair<int, int>> q;

    q.push({start.row, start.col});
    visited[start.row][start.col] = true;

    while (!q.empty()) {
        auto [currentRow, currentCol] = q.front();
        q.pop();

        if (currentRow == end.row && currentCol == end.col) {
            std::vector<Cell> path;
            for (std::pair<int, int> at = {end.row, end.col}; at.first != -1; at = parent[at]) {
                path.push_back(cell_grid[at.first][at.second]);
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

std::unordered_map<std::pair<Cell, Cell>, std::vector<Cell>, pair_hash_cell> 
precompute_shortest_paths(const std::vector<Cell>& cells) 
{
    std::unordered_map<std::pair<Cell, Cell>, std::vector<Cell>, pair_hash_cell> shortestPaths;
    
    for (int i = 0; i < cells.size(); ++i) 
    {
        for (int j = i + 1; j < cells.size(); ++j) 
        {
            if (i == j) continue;
            const Cell& start = cells[i];
            const Cell& end = cells[j];
            std::vector<Cell> path = get_shortest_path(start, end);
            shortestPaths[{start, end}] = path;
            std::reverse(path.begin(), path.end()); // For the reverse path
            shortestPaths[{end, start}] = path;
        }
    }
    
    return shortestPaths;
}

std::vector<Cell> get_global_plan(const Cell& current_cell, const std::vector<Cell> golds, const std::vector<Cell> pickups)
{
    std::vector<Cell> all_cells = {current_cell};
    all_cells.insert(all_cells.end(), golds.begin(), golds.end());
    all_cells.insert(all_cells.end(), pickups.begin(), pickups.end());

    auto shortest_paths = precompute_shortest_paths(all_cells);

    int best_cost = INFINITY;
    std::vector<Cell> best_path;

    std::vector<std::vector<Cell>> permutations = get_all_permutations(golds);
    for (std::vector<Cell> path : permutations)
    {
        // Compute cost of path + current_to_first_gold + last_gold_to_shortest_pickup
        int cost = 0;
        for (int i = 0; i < path.size() - 1; i++)
        {
            cost += shortest_paths[{path[i], path[i+1]}].size();
        }
        cost += shortest_paths[{current_cell, path.front()}].size();
        
        Cell best_endpoint;
        int best_endpoint_cost = INFINITY;
        for (Cell pickup : pickups)
        {   
            int endpoint_cost = shortest_paths[{path.back(), pickup}].size();
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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "global_planner");
    ros::NodeHandle n;
    ros::Rate loop_rate(15);

    ROS_DEBUG("Waiting for Grid Map...");
    map_sub = n.subscribe("grid_map", 1, map_callback);
    while (!map_received)
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    ROS_DEBUG("Map received and processed.");

    ros::ServiceServer execute_plan_srv = n.advertiseService("execute_plan", set_execute_plan_callback);
}