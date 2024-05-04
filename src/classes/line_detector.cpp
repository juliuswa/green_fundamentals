#include "line_detector.h"

int offset_of_laser = 13;

std::list<Vector> LineDetector::get_measurements(const sensor_msgs::LaserScan::ConstPtr& laser_scan) 
{
    std::list<Vector> all_vectors;

    for(int i = 0; i < laser_scan->ranges.size(); i++) {
        float r = laser_scan->ranges[i];
        
        if (r != r || r - m_last_measurement[i] < 0.000001) {
            continue;
        }        

        float theta = i * laser_scan->angle_increment + theta_offset;

        
        Vector vector(r * std::cos(theta), r * std::sin(theta));

        all_vectors.push_back(vector);
    }
    return all_vectors;
}

std::list<Line> LineDetector::find_lines(std::list<Vector> measurements)
{    
    Vector points[measurements.size()];
    std::copy(measurements.begin(), measurements.end(), points);

    std::list<Line> discovered_lines;
    std::set<int> covered;
    
    for (int u = 0; u < measurements.size() - 1; u++) {
        if (covered.count(u)) {
            continue;
        }

        for (int v = u + 1; v < measurements.size(); v++) {
            if (covered.count(v)) {
                continue;
            }     

            Line line(points[u], points[v]);
            std::list<int> matched;

            for (int w = 0; w < measurements.size(); w++) {
                if (covered.count(w)) {
                    continue;
                }

                float distance = line.get_distance_to_point(points[w]);

                if (distance < epsilon) {
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


void LineDetector::detect(const sensor_msgs::LaserScan::ConstPtr& laser_scan) {
    std::list<Vector> measurements = get_measurements(laser_scan);
    std::copy(laser_scan->ranges.begin(), laser_scan->ranges.end(), m_last_measurement);
    ROS_DEBUG("%d measurements taken.", measurements.size()); 

    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    std::list<Line> lines = find_lines(measurements);
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

    ROS_DEBUG("%d lines found. in %ld ms", lines.size(), std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count());    

    Line line_array[lines.size()];
    std::copy(lines.begin(), lines.end(), line_array);

    std::string str;
    Vector robot_direction(1, 0);
    Vector robot_offset(0, 0);

    for (int i = 0; i < lines.size(); ++i) {
        float distance = line_array[i].get_distance_to_point(robot_offset);
        float angle = robot_direction.scalar_product(line_array[i].m_direction);

        ROS_DEBUG("g%d = distance: %f, angle %f°.", i,
            distance, angle);
    }

    ROS_DEBUG("%s", generateSpace(measurements).c_str());
}

LineDetector::LineDetector() {}



