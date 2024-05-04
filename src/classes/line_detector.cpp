#include "line_detector.h"

int offset_of_laser = 13;

std::list<Vector> LineDetector::get_measurements(const sensor_msgs::LaserScan::ConstPtr& laser_scan) 
{
    std::list<Vector> all_vectors;

    for(int i = 0; i < laser_scan->ranges.size(); i++) {
        float r = laser_scan->ranges[i];
        
        if (r != r || r == m_last_measurement[i]) {
            continue;
        }        
theta_offset
        float theta = i * laser_scan->angle_increment + theta_offset;

        Vector vector(r * std::cos(theta), r * std::sin(theta));

        all_vectors.push_back(vector);
    }
    return all_vectors;
}


std::string generateSpace(const std::list<Vector>& points) {
    int width = 125;
    int height = 50;
    
    float dimension = 1.1;

    float stepY = (dimension * 2) / static_cast<float>(width);
    float stepX = (dimension * 2) / static_cast<float>(height);

    std::string grid(width * height, ' ');

    for (const auto& p : points) {
        int xIndex = static_cast<int>((p.x + dimension) / stepX);
        int yIndex = static_cast<int>((p.y + dimension) / stepY);

        grid[xIndex * width + yIndex] = '+';
    }

    std::string result;
    for (int y = 0; y < height; ++y) {
        result += grid.substr(y * width, width) + '\n';
    }

    return result;
}


void LineDetector::detect(const sensor_msgs::LaserScan::ConstPtr& laser_scan) {
    std::list<Vector> measurements = get_measurements(laser_scan);
    ROS_DEBUG("%d measurements taken.", measurements.size()); 

    Vector point_array[measurements.size()];
    std::copy(measurements.begin(), measurements.end(), point_array);

    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    std::list<Line> lines = perform_ransack(point_array, epsilon, min_matches);
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

    ROS_DEBUG("%d lines found. in %ld ms", lines.size(), std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count());    

    Line line_array[lines.size()];
    std::copy(lines.begin(), lines.end(), line_array);

    std::string str;
    Vector robot_direction(1, 0);
    Vector robot_offset(0, 0);

    for (int i = 0; i < lines.size(); ++i) {
        float distance = line_array[i].get_distance_to_point(robot_offset);

        float angle = acos(robot_direction.scalar_product(line_array[i].m_direction) / 
            (robot_direction.get_length() * line_array[i].m_direction.get_length()));

        ROS_DEBUG("g%d = distance: %f, angle %f°.", i,
            distance, angle);
    }
}

LineDetector::LineDetector() {}



