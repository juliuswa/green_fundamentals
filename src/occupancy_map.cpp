#include "ros/ros.h"
#include <cmath>
#include "Eigen/Dense"

#include "green_fundamentals/Grid.h"
#include "green_fundamentals/Row.h"
#include "green_fundamentals/Cell.h"

#include "nav_msgs/OccupancyGrid.h"

#define PIXEL_SIZE 0.01     // m
#define WALL_LENGTH 0.78    // m
#define WALL_THICKNESS 0.02 // m

#define WALL 100
#define FREE 0

ros::Subscriber grid_map_sub;
int** pixel_map = nullptr;
nav_msgs::OccupancyGrid occupancy_grid;

bool map_received = false;

void allocatePixelMap(int height_pixels, int width_pixels)
{
    // Allocate memory for rows
    pixel_map = new int*[height_pixels];

    // Allocate memory for columns in each row
    for (int i = 0; i < height_pixels; ++i)
    {
        pixel_map[i] = new int[width_pixels];
    }

    for (int i = 0; i < height_pixels; i++) 
    {
        for (int j = 0; j < width_pixels; j++) 
        {
            pixel_map[i][j] = FREE;
        }
    }

    // Initialize the OccupancyGrid message
    occupancy_grid.header.frame_id = "map";

    occupancy_grid.info.map_load_time = ros::Time::now();
    occupancy_grid.info.resolution = PIXEL_SIZE; // m/cell
    occupancy_grid.info.height = height_pixels;  // cells
    occupancy_grid.info.width = width_pixels;    // cells

    // Set the origin of the map
    occupancy_grid.info.origin.position.x = 0.0;
    occupancy_grid.info.origin.position.y = 0.0;
    occupancy_grid.info.origin.position.z = 0.0;
    
    occupancy_grid.info.origin.orientation.x = 0.0;
    occupancy_grid.info.origin.orientation.y = 0.0;
    occupancy_grid.info.origin.orientation.z = 0.0;
    occupancy_grid.info.origin.orientation.w = 1.0;
    
    
    occupancy_grid.data.resize(height_pixels * width_pixels);
}

void updateOccupancyGrid()
{
    for (int i = 0; i < occupancy_grid.info.height; ++i)
    {
        for (int j = 0; j < occupancy_grid.info.width; ++j)
        {
            // Convert the 2D pixel_map index to a 1D occupancy_grid index
            int index = i * occupancy_grid.info.width + j;
            occupancy_grid.data[index] = pixel_map[i][j];
        }
    }
}

void swap_rows()
{
    for (int i = 0; i < occupancy_grid.info.height / 2; ++i) {
        int* temp = pixel_map[i];
        pixel_map[i] = pixel_map[occupancy_grid.info.height - 1 - i];
        pixel_map[occupancy_grid.info.height - 1 - i] = temp;
    }
}

void map_callback(const green_fundamentals::Grid::ConstPtr& msg)
{
    int rows = msg->rows.size();
    int cols = msg->rows[0].cells.size();

    // Create row-major array of cells
    std::vector<green_fundamentals::Cell> flat_map;
    for (const auto& row : msg->rows)
    {
        for (const auto& cell : row.cells)
        {
            flat_map.push_back(cell);
        }
    }

    int wall_length_pixels = std::round(WALL_LENGTH / PIXEL_SIZE);
    int corner_pixels = std::round(WALL_THICKNESS / PIXEL_SIZE);
    int complete_line_pixels = 2 * corner_pixels + wall_length_pixels;

    int height_pixels = rows * wall_length_pixels + (rows + 1) * corner_pixels;
    int width_pixels = cols * wall_length_pixels + (cols + 1) * corner_pixels;

    allocatePixelMap(height_pixels, width_pixels);

    std::vector<std::pair<float, float>> cornerpoints;
    for (int y_cell = 0; y_cell < rows; y_cell++) 
    {
        for (int x_cell = 0; x_cell < cols; x_cell++) 
        {
            int corner_y = y_cell * (wall_length_pixels + corner_pixels);
            int corner_x = x_cell * (wall_length_pixels + corner_pixels);
            std::pair<float, float> corner{corner_y, corner_x};
            cornerpoints.push_back(corner);
        }
    }

    // flat_map and cornerpoints should have same order

    for (int n = 0; n < cornerpoints.size(); n++) 
    {
        int y = cornerpoints[n].first;
        int x = cornerpoints[n].second;

        green_fundamentals::Cell cell = flat_map[n];

        for (const auto& wall : cell.walls)
        {
            switch (wall)
                {
                    case green_fundamentals::Cell::LEFT:
                        for (int j = 0; j < corner_pixels; j++)
                        {
                            for (int i = 0; i < complete_line_pixels; i++)
                            {
                                pixel_map[y + i][x + j] = WALL;
                            }
                        }
                        break;
                    case green_fundamentals::Cell::TOP:
                        for (int j = 0; j < corner_pixels; j++)
                        {
                            for (int i = 0; i < complete_line_pixels; i++)
                            {
                                pixel_map[y + j][x + i] = WALL;
                            }
                        }
                        break;
                    case green_fundamentals::Cell::RIGHT:
                        for (int j = 0; j < corner_pixels; j++)
                        {
                            for (int i = 0; i < complete_line_pixels; i++)
                            {
                                pixel_map[y + i][x + j + complete_line_pixels - corner_pixels] = WALL;
                            }
                        }
                        break;
                    case green_fundamentals::Cell::BOTTOM:
                        for (int j = 0; j < corner_pixels; j++)
                        {
                            for (int i = 0; i < complete_line_pixels; i++)
                            {
                                pixel_map[y + j + complete_line_pixels - corner_pixels][x + i] = WALL;
                            }
                        }
                        break;
                    default:
                        ROS_WARN("Unknown wall type: %d", wall);
                        break;
                }
        }

    }

    swap_rows(); // for occupancygrid message

    updateOccupancyGrid();

    map_received = true;

    ROS_INFO("Grid Map received, starting to publish OccupancyMap");

    grid_map_sub.shutdown();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "grid_to_map");
    ros::NodeHandle n;

    ros::Publisher map_pub = n.advertise<nav_msgs::OccupancyGrid>("map", 1);
    grid_map_sub = n.subscribe("grid_map", 1, map_callback);

    ROS_INFO("Starting Node");

    ros::Rate loop_rate(1);  // Hz
    while (ros::ok())
    {
        if (map_received) 
        {
            // Update the timestamp
            occupancy_grid.header.stamp = ros::Time::now();

            // Publish the OccupancyGrid message
            map_pub.publish(occupancy_grid);
        }
        
        ros::spinOnce();
        loop_rate.sleep();
    }
  
    return 0;
}