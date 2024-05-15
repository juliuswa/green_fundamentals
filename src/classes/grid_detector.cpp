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

std::vector<Line> GridDetector::summarize_lines(std::vector<Line> lines) {
    auto compare_by_score = [](const Line& l1, const Line& l2) {
        return l1.m_score > l2.m_score;
    };

    std::sort(lines.begin(), lines.end(), compare_by_score);

    for (int i = 0; i < lines.size(); i++) {
        Eigen::Vector2f polar_representation = lines[i].get_polar_representation();

        ROS_DEBUG("score: %d, distance: %f, theta: %f(%f deg.)", 
            lines[i].m_score, polar_representation[0], polar_representation[1],
            polar_representation[1] * 180 / M_PI);
    }

    return lines;
}

void GridDetector::detect(const sensor_msgs::LaserScan::ConstPtr& laser_scan) {

    ROS_DEBUG("Received LaserScan"); 
    std::list<Eigen::Vector2f> measurements = get_measurements(laser_scan);
    ROS_DEBUG("%ld measurements taken.", measurements.size()); 
    // ROS_DEBUG("%s", generateSpace(measurements).c_str());

    ROS_DEBUG("%ld measurements taken.", measurements.size()); 

    Eigen::Vector2f point_array[measurements.size()];
    std::copy(measurements.begin(), measurements.end(), point_array);
    std::vector<Line> lines = perform_ransac(point_array, measurements.size());   

    summarize_lines(lines);  
}

GridDetector::GridDetector() {}



