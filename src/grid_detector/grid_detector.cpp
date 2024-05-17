#include "grid_detector.h"

void GridDetector::get_measurements(const sensor_msgs::LaserScan::ConstPtr& laser_scan) 
{
    ROS_DEBUG("Received LaserScan");   
    m_current_measurement.clear();

    for(int i = 0; i < laser_scan->ranges.size(); i++) {
        float r = laser_scan->ranges[i];
        
        if (r != r || r == m_raw_last_measurement[i]) {
            continue;
        }

        m_raw_last_measurement[i] = r;      

        float theta = i * laser_scan->angle_increment + theta_offset;

        Eigen::Vector2f vector {(r * std::cos(theta)) + x_offset, r * std::sin(theta)};

        m_current_measurement.push_back(vector);
    }

    ROS_DEBUG("%ld measurements taken.", m_current_measurement.size()); 
    return;
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

GridDetectorResponse GridDetector::create_grid_detector_response(std::vector<Line> lines) {
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

    if ((best_angle - M_PI / 2) > 20) {
        Eigen::Vector2f default_offset {0.0, 0.0};
        GridDetectorResponse response = {false, default_offset, 0.0};
        return response;
    }

    ROS_INFO("best angle is %f", best_angle * 180 / M_PI);

    Eigen::Vector2f cut_vertex = lines[line1].get_cut_vertex(lines[line2]);
    ROS_INFO("cut vertex = (%f, %f)", cut_vertex[0], cut_vertex[1]);

    float direction1_factor = lines[line1].m_direction[0] < 0.0 ? 0.39 : -0.39;
    float direction2_factor = lines[line2].m_direction[0] < 0.0 ? 0.39 : -0.39;

    Eigen::Vector2f midpoint = cut_vertex + lines[line1].m_direction * direction1_factor + lines[line2].m_direction * direction2_factor;    
    Eigen::Vector2f polar_line1 = lines[line1].get_polar_representation();

    GridDetectorResponse response = {true, midpoint, polar_line1[1]};
    return response;
}

GridDetectorResponse GridDetector::detect() {
    ROS_DEBUG("detect");   

    // ROS_DEBUG("%s", generateSpace(measurements).c_str()); 

    if (m_current_measurement.size() < 50) {
        ROS_WARN("too few measurements for GridDetector::detect()");

        Eigen::Vector2f default_offset {0.0, 0.0};
        GridDetectorResponse response = {false, default_offset, 0.0};
        return response;
    }

    std::vector<Line> lines = perform_ransac(m_current_measurement); 
    ROS_DEBUG("%d lines detected by ransack", lines.size());   

    for (int i = 0; i < lines.size(); i++) {
        Eigen::Vector2f polar_line = lines[i].get_polar_representation();
        ROS_DEBUG("%d: dist: %f, theta: %f",
            i, polar_line[0], polar_line[1] * 180 / M_PI);
    }

    GridDetectorResponse response = create_grid_detector_response(lines);
    ROS_INFO("response: success = %d, offset = (%f, %f) theta = %f", 
        response.success, 
        response.offset[0], response.offset[1],
        response.theta * 180 / M_PI); 

    return response;
}

GridDetector::GridDetector() {}



