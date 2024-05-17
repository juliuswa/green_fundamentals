#include <cstdlib>
#include <math.h>
#include <list>
#include <set>
#include <chrono>

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "green_fundamentals/DetectGrid.h"

#include "../algorithms/ransac.cpp"


const float theta_offset = - 2.37; 
const float x_offset = 0.13;

std::list<Eigen::Vector2f> m_current_measurement;
float m_raw_last_measurement[2000];

void get_measurements(const sensor_msgs::LaserScan::ConstPtr& laser_scan) 
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

void write_detect_grid_response(std::vector<Line> lines, green_fundamentals::DetectGrid::Response &res) {
    std::vector<Eigen::Vector2f> polar_lines;

    for (int i = 0; i < lines.size(); i++) {
        Eigen::Vector2f polar_line = lines[i].get_polar_representation();
        polar_lines.push_back(polar_line);
        
        ROS_DEBUG("%d: dist: %f, theta: %f",
            i, polar_line[0], polar_line[1] * 180 / M_PI);
    }

    int line1;
    int line2;
    float best_angle = 0.0;

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

    if (std::abs(best_angle - M_PI / 2) > 0.3) {
        ROS_WARN("no 90 degrees angle detected.");
        return;
    }

    ROS_INFO("best angle is %f", best_angle * 180 / M_PI);

    Eigen::Vector2f cut_vertex = lines[line1].get_cut_vertex(lines[line2]);
    ROS_INFO("cut vertex = (%f, %f)", cut_vertex[0], cut_vertex[1]);

    Eigen::Vector2f line1_direction = lines[line1].m_direction;
    Eigen::Vector2f line2_direction = lines[line2].m_direction;
    float factor = 0.39;

    Eigen::Vector2f midpoint_versions[4];
    midpoint_versions[0] = cut_vertex + line1_direction * factor + line2_direction * factor;
    midpoint_versions[1] = cut_vertex + line1_direction * factor - line2_direction * factor;
    midpoint_versions[2] = cut_vertex - line1_direction * factor + line2_direction * factor;
    midpoint_versions[3] = cut_vertex - line1_direction * factor - line2_direction * factor;

    float closest_distance = 1.0;
    int closest_index = 4;

    for (int i = 0; i < 4; i++) {
        float distance = midpoint_versions[i].norm();

        if (distance < closest_distance) {
            closest_distance = distance;
            closest_index = i;
        }
    }

    if (closest_index > 4) {
        ROS_WARN("no midpoint_versions identified.");
        return;
    }

    Eigen::Vector2f cell_center = midpoint_versions[closest_index];    
    Eigen::Vector2f polar_line1 = lines[line1].get_polar_representation();

    float mod_value = M_PI / 2;
    float theta_offset = polar_line1[1] - std::round(polar_line1[1] / mod_value) * mod_value;

    ROS_INFO("cell center = (%f, %f)", cell_center[0], cell_center[1]);

    res.success = true;
    res.x_offset = cell_center[0];
    res.y_offset = cell_center[1];
    res.theta_offset = theta_offset;

    return;
}

bool detect_grid(green_fundamentals::DetectGrid::Request  &req, green_fundamentals::DetectGrid::Response &res)
{
    ROS_INFO("accepted grid detection request.");

    res.success = false;
    res.x_offset = 0.0;
    res.y_offset = 0.0;
    res.theta_offset = 0.0;

    if (m_current_measurement.size() < 50) {
        ROS_WARN("too few measurements for GridDetector::detect()");
        return false;
    }

    std::vector<Line> lines = perform_ransac(m_current_measurement); 
    ROS_DEBUG("%d lines detected by ransack", lines.size());   

    if (lines.size() < 2) {
        ROS_WARN("too few lines detected.");
        return false;
    }

    write_detect_grid_response(lines, res);

    ROS_INFO("response: x_offset=%f, y_offset=%f, theta_offset=%f deg.", res.x_offset, res.y_offset, res.theta_offset * 180 / M_PI);
    return res.success;
}

int main(int argc, char **argv)
{  
    ros::init(argc, argv, "grid_detection_server");
    ros::NodeHandle n;

    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info)) {
        ros::console::notifyLoggerLevelsChanged();
    }

    ros::Subscriber sub = n.subscribe("scan_filtered", 1, get_measurements);
    ros::ServiceServer service = n.advertiseService("detect_grid", detect_grid);

    ROS_INFO("Ready to detect_grid");
    ros::spin();

    return 0;
}