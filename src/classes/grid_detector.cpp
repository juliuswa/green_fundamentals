#include "grid_detector.h"

std::list<Eigen::Vector2f> GridDetector::get_measurements(const sensor_msgs::LaserScan::ConstPtr& laser_scan) 
{
    std::list<Eigen::Vector2f> all_vectors;

    for(int i = 0; i < laser_scan->ranges.size(); i++) {
        float r = laser_scan->ranges[i];
        
        if (r != r || r == m_last_measurement[i]) {
            continue;
        }        

        float theta = i * laser_scan->angle_increment + theta_offset;

        Eigen::Vector2f vector {(r * std::cos(theta)) + x_offset, r * std::sin(theta)};

        all_vectors.push_back(vector);
    }
    return all_vectors;
}

std::string generateSpace(const std::list<Eigen::Vector2f>& points) {
    int width = 125;
    int height = 50;
    
    float dimension = 1.1;

    float stepY = (dimension * 2) / static_cast<float>(width);
    float stepX = (dimension * 2) / static_cast<float>(height);

    std::string grid(width * height, ' ');

    for (const auto& p : points) {
        int xIndex = static_cast<int>((p[0] + dimension) / stepX);
        int yIndex = static_cast<int>((p[1] + dimension) / stepY);

        grid[(height - xIndex) * width + (width - yIndex)] = '+';
    }

    std::string result;
    for (int y = 0; y < height; ++y) {
        result += grid.substr(y * width, width) + '\n';
    }

    return result;
}

Eigen::Vector2f GridDetector::find_cell_midpoint(std::vector<Line> lines) {
    std::vector<Eigen::Vector2f> polar_lines;

    for (int i = 0; i < lines.size(); i++) {
        polar_lines.push_back(lines[i].get_polar_representation());
    }

    int line1;
    int line2;
    float best_angle = 0;

    for (int i = 0; i < polar_lines.size(); i++)  {
        for (int j = i + 1; j < polar_lines.size(); j++)  {
            float angle = std::abs(polar_lines[i][1] - polar_lines[j][1]);

            if (std::abs(angle - M_PI / 2) < std::abs(best_angle - M_PI / 2)) {
                line1 = i;
                line2 = j;
                best_angle = angle;
            }
        }
    }

    ROS_INFO("best angle is %f", best_angle * 180 / M_PI);

    Eigen::Vector2f cut_vertex = lines[line1].get_cut_vertex(lines[line2]);
    Eigen::Vector2f midpoint_offset {0.39, 0.39};

    return cut_vertex - midpoint_offset;
}

void GridDetector::detect(const sensor_msgs::LaserScan::ConstPtr& laser_scan) {

    ROS_DEBUG("Received LaserScan"); 
    std::list<Eigen::Vector2f> measurements = get_measurements(laser_scan);
    ROS_DEBUG("%ld measurements taken.", measurements.size()); 
    // ROS_DEBUG("%s", generateSpace(measurements).c_str());

    ROS_DEBUG("%ld measurements taken.", measurements.size()); 

    std::vector<Line> lines = perform_ransac(measurements); 
    ROS_DEBUG("%d: lines detected by ransack", lines.size());   

    for (int i = 0; i < lines.size(); i++) {
        Eigen::Vector2f polar_line = lines[i].get_polar_representation();
        ROS_DEBUG("%d: dist: %f, theta: %f",
            i, polar_line[0], polar_line[1] * 180 / M_PI);
    }

    Eigen::Vector2f midpoint_delta = find_cell_midpoint(lines);
    ROS_INFO("midpoint delta: (%f, %f)", midpoint_delta[0], midpoint_delta[1]);      
}

GridDetector::GridDetector() {}



