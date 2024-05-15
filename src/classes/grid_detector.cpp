#include "grid_detector.h"

float offset_of_laser = 0.13;

std::list<Eigen::Vector2f> GridDetector::get_measurements(const sensor_msgs::LaserScan::ConstPtr& laser_scan) 
{
    std::list<Eigen::Vector2f> all_vectors;

    for(int i = 0; i < laser_scan->ranges.size(); i++) {
        float r = laser_scan->ranges[i];
        
        if (r != r || r == m_last_measurement[i]) {
            continue;
        }        

        float theta = i * laser_scan->angle_increment + theta_offset;

        Eigen::Vector2f vector {r * std::cos(theta), r * std::sin(theta)};

        all_vectors.push_back(vector);
    }
    return all_vectors;

    // std::list<Vector> all_vectors;

    // int center_index = 400;
    // float index_range = (M_PI / 2.) / laser_scan->angle_increment;  // nur 180° anschauen

    // for (int angle_offset = 0; angle_offset <= index_range ; angle_offset++) {  // linke Seite von 0° bis 90°
    //     int index = center_index + angle_offset; 
    //     if(laser_scan->ranges[index] != laser_scan->ranges[index]) continue;  // nan

    //     float r_laser = laser_scan->ranges[index];  // these are in relation to laser
    //     float theta_laser = angle_offset * laser_scan->angle_increment;

    //     float x = offset_of_laser + (r_laser * std::cos(theta_laser));  // these are to (0, 0)
    //     float y = r_laser * std::sin(theta_laser);

    //     Vector vector(x, y);
    //     // ROS_INFO("LeftMeas: point(np.array([%f, %f])", vector.x, vector.y);

    //     all_vectors.push_back(vector);
    // }

    // for (int angle_offset = 1; angle_offset <= index_range ; angle_offset++) {
    //     int index = center_index - angle_offset; 
    //     if(laser_scan->ranges[index] != laser_scan->ranges[index]) continue;  // nan

    //     float r_laser = laser_scan->ranges[index];  // these are in relation to laser
    //     float theta_laser = (2 * M_PI) - angle_offset * laser_scan->angle_increment;  // 360° - the angle from x-axis

    //     float x = offset_of_laser + (r_laser * std::cos(theta_laser));  // these are to (0, 0)
    //     float y = r_laser * std::sin(theta_laser);

    //     Vector vector(x, y);
    //     // ROS_INFO("RightMeaas: point(np.array([%f, %f])", vector.x, vector.y);

    //     all_vectors.push_back(vector);
    // }

    // return all_vectors;
}

std::string generateSpace(const std::list<Eigen::Vector2f>& points) {
    int width = 150;
    int height = 60;
    
    float dimension = 1.1;

    float stepY = (dimension * 2) / static_cast<float>(width);
    float stepX = (dimension * 2) / static_cast<float>(height);

    std::string grid(width * height, ' ');

    for (const auto& p : points) {
        int xIndex = static_cast<int>((p[0] + dimension) / stepX);
        int yIndex = static_cast<int>((p[1] + dimension) / stepY);

        grid[xIndex * width + yIndex] = '+';
    }

    std::string result;
    for (int y = 0; y < height; ++y) {
        result += grid.substr(y * width, width) + '\n';
    }

    return result;
}

void GridDetector::detect(const sensor_msgs::LaserScan::ConstPtr& laser_scan) {
    // if (received_packet) {
    //     return;
    // }

    ROS_DEBUG("Received LaserScan"); 
    std::list<Eigen::Vector2f> measurements = get_measurements(laser_scan);
    ROS_DEBUG("%ld measurements taken.", measurements.size()); 
    ROS_DEBUG("%s", generateSpace(measurements).c_str());

    Eigen::Vector2f point_array[measurements.size()];
    std::copy(measurements.begin(), measurements.end(), point_array);

    ROS_DEBUG("Starting RANSAC"); 
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    std::vector<Line> lines = perform_ransac(point_array, measurements.size());
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

    ROS_DEBUG("%ld lines found. in %ld ms", lines.size(), std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count());    
    
    for (int i = 0; i < lines.size(); i++) {
        ROS_DEBUG("score: %d, offset: (%f, %f), direction: (%f, %f)", lines[i].m_score,
            lines[i].m_offset[0], lines[i].m_offset[1], 
            lines[i].m_direction[0], lines[i].m_direction[1]);

    }
    
    // Line line_array[lines.size()];
    // std::copy(lines.begin(), lines.end(), line_array);

    // ROS_DEBUG("Starting Midpoint calculation"); 
    // mid_and_ori = get_midpoint_from_lines(lines, Vector(0., 0.));
    // ROS_DEBUG("Midpoint calculation ended"); 
    // ROS_DEBUG("Midpoint at: (%f, %f)", mid_and_ori.first.x, mid_and_ori.first.y); 

    // std::string str;
    // Vector robot_direction(1, 0);
    // Vector robot_offset(0, 0);

    // // for (int i = 0; i < lines.size(); ++i) {
    // //     float distance = line_array[i].get_distance_to_point(robot_offset);

    // //     float angle = acos(robot_direction.scalar_product(line_array[i].m_direction) / 
    // //         (robot_direction.get_length() * line_array[i].m_direction.get_length()));

    // //     ROS_DEBUG("g%d = distance: %f, angle %f°.", i,
    // //         distance, angle);
    // // }

    // received_packet = true;
}

GridDetector::GridDetector() {}



