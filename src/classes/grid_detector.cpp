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

Eigen::Vector2f GridDetector::summarize_group(std::vector<Eigen::Vector2f> lines) {
    Eigen::Vector2f sum_vector {0.0, 0.0};
    
    for (int i = 0; i < lines.size(); i++) {
        sum_vector = sum_vector + lines[i];
    }

    return sum_vector / (float)lines.size();
}

std::vector<Eigen::Vector2f> GridDetector::summarize_lines(std::vector<Line> lines) {
    std::vector<Eigen::Vector2f> polar_lines;

    for (int i = 0; i < lines.size(); i++) {
        polar_lines.push_back(lines[i].get_polar_representation());
    }

    auto compare_by_theta = [](const Eigen::Vector2f& v1, const Eigen::Vector2f& v2) {
        return v1[1] > v2[1];
    };

    std::sort(polar_lines.begin(), polar_lines.end(), compare_by_theta);

    const float radian_epsilon = 0.1;

    std::vector<Eigen::Vector2f> first_group;
    first_group.push_back(polar_lines[0]);
    std::vector<std::vector<Eigen::Vector2f>> groups;
    groups.push_back(first_group);
    int current_group = 0;

    for (int i = 1; i < polar_lines.size(); i++) {
        int last_current_group_index = groups[current_group].size();

        if (groups[current_group][last_current_group_index][1] + epsilon > polar_lines[i][1]) {
            groups[current_group].push_back(polar_lines[i]);
            continue;
        }
        
        std::vector<Eigen::Vector2f> new_group;
        new_group.push_back(polar_lines[i]);
        groups.push_back(new_group);
        current_group += 1;        
    }
    
    std::vector<Eigen::Vector2f> final_lines;

    for (int i = 0; i < groups.size(); i++) {
        final_lines.push_back(summarize_group(groups[i]));  
    }

    return final_lines;
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

    std::vector<Eigen::Vector2f> final_lines = summarize_lines(lines);  

    for (int i = 0; i < final_lines.size(); i++) {
        ROS_DEBUG("%d: dist: %f, theta: %f",
            i, final_lines[i][0], final_lines[i][1] * 180 / M_PI);
    }
}

GridDetector::GridDetector() {}



