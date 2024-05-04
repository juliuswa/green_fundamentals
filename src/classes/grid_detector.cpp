#include "grid_detector.h"

int offset_of_laser = 13;

std::list<Vector> GridDetector::get_measurements(const sensor_msgs::LaserScan::ConstPtr& laser_scan) 
{
    std::list<Vector> all_vectors;

    for(int i = 0; i < laser_scan->ranges.size(); i++) {
        float r = laser_scan->ranges[i];
        float theta = i * laser_scan->angle_increment + theta_offset;

        if (r != r) {
            continue;
        }
        
        Vector vector(r * std::cos(theta), r * std::sin(theta));

        all_vectors.push_back(vector);
    }

    return all_vectors;
}

std::list<Line> GridDetector::find_lines(std::list<Vector> measurements)
{    
    Vector points[measurements.size()];
    std::copy(measurements.begin(), measurements.end(), points);

    std::list<Line> discovered_lines;
    std::set<int> covered;
    
    for (int u = 0; u < measurements.size() - 1; u++) {
        if (covered.count(u)) {
            continue;
        }
        
        if (u % 1 == 0) {
            ROS_DEBUG("%d", u);
        }

        for (int v = u + 1; v < measurements.size(); v++) {
            if (covered.count(v)) {
                continue;
            }   

            if (v % 1 == 0) {
                ROS_DEBUG("u %d", v);
            }         

            Line line(points[u], points[v]);
            
            std::list<int> matched;

            for (int w = 0; w < measurements.size(); w++) {
                if (covered.count(w)) {
                    continue;
                }

                
                if (line.get_distance_to_point(points[w], epsilon) < epsilon) {
                    matched.push_back(w);
                }
                
            }

            if (matched.size() < min_matches) {
                continue;
            }

            discovered_lines.push_back(line);
            covered.insert(matched.begin(), matched.end());
        }
    }

    return discovered_lines;
}


std::string generateSpace(const std::list<Vector>& points) {
    int width = 125;
    int height = 50;
    
    float dimension = 1.1;

    float stepX = (dimension * 2) / static_cast<float>(width);
    float stepY = (dimension * 2) / static_cast<float>(height);

    std::string grid(width * height, ' ');

    for (const auto& p : points) {
        int xIndex = static_cast<int>((p.x + dimension) / stepX);
        int yIndex = static_cast<int>((p.y + dimension) / stepY);

        grid[yIndex * width + xIndex] = 'X';
    }

    std::string result;
    for (int y = 0; y < height; ++y) {
        result += grid.substr(y * width, width) + '\n';
    }

    return result;
}


void GridDetector::detect_grid(const sensor_msgs::LaserScan::ConstPtr& laser_scan) {
    ROS_INFO("detect_grid is called.");

    std::list<Vector> measurements = get_measurements(laser_scan);
    std::list<Line> lines = find_lines(measurements);
    
    /*
    Line* closest_line = nullptr;
    float closest_distance = 1.0;

    Vector robot(-13.0, 0.0);

    for (Line line : lines) {
        float distance = line.get_distance_to_point(robot, epsilon);

        if (distance < closest_distance) {
            closest_line = &line;
            closest_distance = distance;
        }
    }

    ROS_DEBUG("%d lines found. Closest distance = %f", lines.size(), closest_distance);
    */

    ROS_DEBUG("%s", generateSpace(measurements).c_str());
}

GridDetector::GridDetector() {}



