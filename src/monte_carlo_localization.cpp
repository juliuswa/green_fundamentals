#include "ros/ros.h"
#include <boost/bind.hpp>
#include <cmath>
#include <random>
#include "Eigen/Dense"
#include <tf2/LinearMath/Quaternion.h>

#include "robot_constants.h"

#include "nav_msgs/OccupancyGrid.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Point32.h"
#include "sensor_msgs/PointCloud.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseArray.h"
#include "green_fundamentals/Position.h"
#include "create_fundamentals/SensorPacket.h"

#define SUBSAMPLE_LASERS 32
#define NUM_PARTICLES 2000
#define NUM_RANDOM_PARTICLES 100
#define RAY_STEP_SIZE 0.01

#define RESAMPLE_STD_POS 0.02
#define RESAMPLE_STD_THETA 0.04

struct Particle {
    Eigen::Vector2f position;
    float theta;
    float weight = 0.;

    Particle() = default;

    Particle(const Eigen::Vector2f& pos, float t, float w) 
        : position(pos), theta(t), weight(w) {}

    Particle(const Particle& other)
        : position(other.position), theta(other.theta), weight(other.weight) {}

    Particle& operator=(const Particle& other) {
        if (this != &other) {
            position = other.position;
            theta = other.theta;
            weight = other.weight;
        }

        return *this;
    }
};

// Receive Map
ros::Subscriber map_sub;
bool map_received = false;

// Map data
std::vector<std::vector<int8_t>> map_data;
int map_height, map_width;

// Localization parameters
float x_min = 0.0; // m
float x_max = 3.0;
float y_min = 0.0;
float y_max = 3.0;

float last_left = 0.;
float last_right = 0.;

float current_left = 0.;
float current_right = 0.;

bool is_first_encoder_measurement = true;

// Relative motion since the last time particles were updated
float dx = 0.;
float dy = 0.;
float dtheta = 0.;

const Eigen::Vector2f laser_offset{0.13, 0.};

std::default_random_engine generator;

Particle particles[NUM_PARTICLES];
Particle new_particles[NUM_PARTICLES];

ros::Publisher pose_pub, posearray_pub, actual_ray_pub, expected_ray_pub;

std::pair<int, int> metric_to_grid_index(float x, float y) 
{
    int gx = floor(x * 100);
    int gy = floor(y * 100);
    int row = std::min(std::max(gy, 0), map_height -1);
    int col = std::min(std::max(gx, 0), map_width -1);

    return {row, col};
}

void sensor_callback(const create_fundamentals::SensorPacket::ConstPtr& msg)
{    
    current_left = msg->encoderLeft;
    current_right = msg->encoderRight;

    if (is_first_encoder_measurement) {
        last_left = current_left;
        last_right = current_right;

        is_first_encoder_measurement = false;
    }
}

void map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{    
    map_height = msg->info.height;
    map_width = msg->info.width;

    // Resize the map_data to match the map dimensions
    map_data.resize(map_height);
    for (int i = 0; i < map_height; ++i) {
        map_data[i].resize(map_width);
    }

    // Fill the 2D array with the occupancy data
    for (int y = 0; y < map_height; ++y) {
        for (int x = 0; x < map_width; ++x) {
            // Calculate the index in the 1D data array
            int index = x + y * map_width;
            map_data[y][x] = msg->data[index];
        }
    }

    ROS_INFO("Map received");
    map_received = true;
    map_sub.shutdown();
}

Particle get_random_particle() 
{
    std::uniform_real_distribution<float> uni_dist(0., 1.);

    float x = uni_dist(generator) * (x_max - x_min);
    float y = uni_dist(generator) * (y_max - y_min);

    float theta = uni_dist(generator) * (2 * M_PI);
    Eigen::Vector2f pos{x, y};
    Particle particle{pos, theta, 0.};
    return particle;
}

void init_particles()
{
    for (int i = 0; i < NUM_PARTICLES; i++)
    {
        particles[i] = get_random_particle();
    }
}

void update_particles()
{
    float distance_left = (current_left - last_left) * WHEEL_RADIUS; // m
    float distance_right = (current_right - last_right) * WHEEL_RADIUS; // m
    float distance = (distance_left + distance_right) / 2;

    float delta_theta = (distance_right - distance_left) / WHEEL_BASE;

    std::default_random_engine generator;
    std::normal_distribution<float> normal_dist_pos(1., RESAMPLE_STD_POS);
    std::normal_distribution<float> normal_dist_theta(1., RESAMPLE_STD_THETA);

    for (int i = 0; i < NUM_PARTICLES; i++)
    {   
        float x_delta = distance * cos(particles[i].theta + delta_theta/2);
        float y_delta = distance * sin(particles[i].theta + delta_theta/2);

        particles[i].position[0] += x_delta * normal_dist_pos(generator);
        particles[i].position[1] += y_delta * normal_dist_pos(generator);
        particles[i].theta += delta_theta * normal_dist_theta(generator);
    }
}

void evaluate_particles(const sensor_msgs::LaserScan::ConstPtr& msg) 
{
    for (int p = 0; p < NUM_PARTICLES; p++)
    {
        ROS_DEBUG("particle %d: x=%f, y=%f, th=%f", p, particles[p].position[0], particles[p].position[1], particles[p].theta);
        // TODO: Add laser offset

        float total_delta = 0.;

        /*
        sensor_msgs::PointCloud expected_ray_points;
        sensor_msgs::PointCloud actual_ray_points;
        */

        for (int i = 0; i < SUBSAMPLE_LASERS; i++) 
        {
            // ROS_DEBUG("laser: %d", i);

            int index = i * msg->ranges.size() / SUBSAMPLE_LASERS;
            
            float real_distance = msg->ranges[index];
            
            if (real_distance != real_distance) {
                real_distance = 1.0;
            }            
            else if (real_distance < RAY_STEP_SIZE) {
                real_distance = RAY_STEP_SIZE;
            }

            // Get laser angle
            float angle = msg->angle_min + msg->angle_increment * index;
            float ray_angle = particles[p].theta + angle;

            float r = RAY_STEP_SIZE;
            
            float ray_x = 0.;
            float ray_y = 0.;

            // ROS_DEBUG("ray casting");
            while (r < 1.0) 
            {
                ray_x = particles[p].position[0] + r * std::cos(ray_angle);
                ray_y = particles[p].position[1] + r * std::sin(ray_angle);

                if (ray_x > x_max || ray_x < x_min || ray_y > y_max || ray_y < y_min) break;

                std::pair<int, int> grid_index = metric_to_grid_index(ray_x, ray_y);

                if (map_data[grid_index.first][grid_index.second] != 0) break;

                r += RAY_STEP_SIZE;
            }

            /*
            float actual_ray_x = particles[p].position[0] + real_distance * std::cos(ray_angle);
            float actual_ray_y = particles[p].position[1] + real_distance * std::sin(ray_angle);
            
            geometry_msgs::Point32 actual_p;
            actual_p.x = actual_ray_x;
            actual_p.y = actual_ray_y;
            actual_p.z = 0.01;
            actual_ray_points.points.push_back(actual_p);

            geometry_msgs::Point32 expected_p;
            expected_p.x = ray_x;
            expected_p.y = ray_y;
            expected_p.z = 0.01;
            expected_ray_points.points.push_back(expected_p);
            */

            // ROS_DEBUG("index %d, distance: %f, r: %f", i, real_distance, r);

            total_delta += fabs(r - real_distance);            
        }

        /*
        actual_ray_points.header.frame_id = "map";
        actual_ray_pub.publish(actual_ray_points);

        expected_ray_points.header.frame_id = "map";
        expected_ray_pub.publish(expected_ray_points);
        */

        particles[p].weight = std::exp(-total_delta);
    }
}

void resample_particles()
{
    float max_weight = 0.;

    for (int i = 0; i < NUM_PARTICLES; i++)
    {
        if (particles[i].weight > max_weight)
        {
            max_weight = particles[i].weight;
        }
    }

    ROS_INFO("max weight: %f", max_weight);

    std::default_random_engine generator;
    std::uniform_real_distribution<float> uni_dist(0., 1.);

    int index = uni_dist(generator) * NUM_PARTICLES;
    std::normal_distribution<float> normal_dist_pos(0., RESAMPLE_STD_POS / 2);
    std::normal_distribution<float> normal_dist_theta(0., RESAMPLE_STD_THETA / 2);
    
    double beta = 0.0;
    
    ROS_DEBUG("resampling... ");
    for (int i = 0; i < NUM_PARTICLES - NUM_RANDOM_PARTICLES; ++i)
    {
        beta += uni_dist(generator) * 2 * max_weight;

        while (beta > particles[index].weight)
        {
            beta -= particles[index].weight;
            index = (index + 1) % NUM_PARTICLES;
        }

        new_particles[i] = particles[index];
        new_particles[i].position[0] += normal_dist_pos(generator);
        new_particles[i].position[1] += normal_dist_pos(generator);
        new_particles[i].theta += normal_dist_theta(generator);
    }

    for (int i = NUM_PARTICLES - NUM_RANDOM_PARTICLES; i < NUM_PARTICLES; ++i)
    {
        new_particles[i] = get_random_particle();
    }

    ROS_DEBUG("copying... ");

    for (int i = 0; i < NUM_PARTICLES; i++) 
    {
        particles[i] = new_particles[i];
    }    
}

geometry_msgs::Pose particle_to_pose(int particle_index)
{
    geometry_msgs::Pose pose;

    pose.position.x = particles[particle_index].position[0];
    pose.position.y = particles[particle_index].position[1];
    pose.position.z = 0.05;

    tf2::Quaternion quaternion;
    quaternion.setRPY(0, 0, particles[particle_index].theta);
    pose.orientation.x = quaternion.x();
    pose.orientation.y = quaternion.y();
    pose.orientation.z = quaternion.z();
    pose.orientation.w = quaternion.w();

    return pose;
}

void publish_particles()
{
    // Find best particle
    float max_weight = 0.;
    int index = -1;
    for (int i = 0; i < NUM_PARTICLES; i++)
    {
        if (particles[i].weight > max_weight)
        {
            max_weight = particles[i].weight;
            index = i;
        }
    }

    ROS_DEBUG("Best weight: %f", max_weight);

    pose_pub.publish(particle_to_pose(index));

    geometry_msgs::PoseArray pose_array;
    pose_array.header.frame_id = "map";

    for (int i = 0; i < NUM_PARTICLES; i++)
    {
        pose_array.poses.push_back(particle_to_pose(i));
    }
    
    posearray_pub.publish(pose_array);
}

void laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    ROS_DEBUG("update_particles");
    update_particles();

    ROS_DEBUG("evaluate_particles");
    evaluate_particles(msg);

    ROS_DEBUG("publish_particles");
    publish_particles();

    ROS_DEBUG("resample_particles");
    resample_particles();
    
    last_left = current_left;
    last_right = current_right;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mc_localization");
    ros::NodeHandle n;

    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info)) {
        ros::console::notifyLoggerLevelsChanged();
    }
    
    map_sub = n.subscribe("map", 1, map_callback);

    ROS_INFO("Starting node.");
    
    ros::Rate loop_rate(10);
    while (!map_received)
    {
        ros::spinOnce();

        loop_rate.sleep();
    }

    ROS_INFO("Map received.");
    
    init_particles();

    ros::Subscriber odo_sub = n.subscribe("sensor_packet", 1, sensor_callback);
    ros::Subscriber laser_sub = n.subscribe("scan_filtered", 1, laser_callback);

    /*
    actual_ray_pub = n.advertise<sensor_msgs::PointCloud>("actual_ray", 1);
    expected_ray_pub = n.advertise<sensor_msgs::PointCloud>("expected_ray", 1);
    */

    pose_pub = n.advertise<geometry_msgs::Pose>("pose", 1);
    posearray_pub = n.advertise<geometry_msgs::PoseArray>("pose_array", 1);

    ros::spin();

    return 0;
}