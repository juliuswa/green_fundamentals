#include "ros/ros.h"
#include <cmath>
#include <random>
#include <mutex>
#include "Eigen/Dense"

#include <tf2/LinearMath/Quaternion.h>
#include <vector>

#include "robot_constants.h"

#include "nav_msgs/OccupancyGrid.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseArray.h"
#include "green_fundamentals/Position.h"
#include "green_fundamentals/Pose.h"
#include "create_fundamentals/SensorPacket.h"

// MCL Algorithm
const int SUBSAMPLE_LASERS = 32;
const int NUM_PARTICLES = 1000;
const float RAY_STEP_SIZE = 0.01;

// Motion Model 
const float RESAMPLE_STD_POS = 0.03;
const float RESAMPLE_STD_THETA = 5 * M_PI/180.0;

// Flags
bool converged = false;
bool force_update = true; // first localization without movement needed
bool map_received = false;
bool laser_received = false;
bool sensor_received = false;
bool is_first_encoder_measurement = true;

// Particles
struct Particle {
    float x, y, theta, weight;
};
std::vector<Particle> particles;

// Lasers
std::vector<std::pair<float, float>> laser_data;

// Map
ros::Subscriber map_sub;
std::vector<std::vector<int8_t>> map_data;
int map_height, map_width;
float x_max, y_max; // m

// Encoder values
float last_left = 0.;
float last_right = 0.;
float current_left = 0.;
float current_right = 0.;
float delta_dist = 0.;
float delta_theta = 0.;

// Random generators
std::default_random_engine generator;
std::uniform_real_distribution<float> uniform_dist(0., 1.);
std::normal_distribution<float> normal_dist_pos(0., RESAMPLE_STD_POS);
std::normal_distribution<float> normal_dist_theta(0., RESAMPLE_STD_THETA);

// Publishers
ros::Publisher particle_array_viz_pub, position_pub;

// ############### HELPERS ###############
std::pair<int, int> metric_to_grid_index(float x, float y) 
{
    int gx = floor(x * 100);
    int gy = floor(y * 100);
    int row = std::min(std::max(gy, 0), map_height -1);
    int col = std::min(std::max(gx, 0), map_width -1);

    return {row, col};
}

Particle get_random_particle(float init_weight) 
{
    float x, y;
    std::pair<int, int> grid_index;
    do {
        x = uniform_dist(generator) * x_max;
        y = uniform_dist(generator) * y_max;
        grid_index = metric_to_grid_index(x, y);
    } while (map_data[grid_index.first][grid_index.second] != 0);
    
    float theta = uniform_dist(generator) * (2 * M_PI) - M_PI;

    Particle particle{x, y, theta, init_weight};
    return particle;
}

float normalize_angle(float angle)
{
    angle = fmod(angle, (2 * M_PI));
    if (angle > M_PI)
        angle -= 2 * M_PI;

    return angle;
}

float get_particle_weight(const Particle& particle) 
{
    float laser_x =  particle.x + 0.13 * std::cos(particle.theta);
    float laser_y =  particle.y + 0.13 * std::sin(particle.theta);

    {
        if (particle.x > x_max || particle.x < 0 || particle.y > y_max || particle.y < 0) return 0.;
        if (laser_x > x_max || laser_x < 0 || laser_y > y_max || laser_y < 0) return 0.;
        std::pair<int, int> grid_index = metric_to_grid_index(particle.x, particle.y);
        if (map_data[grid_index.first][grid_index.second] != 0) return 0.;
        grid_index = metric_to_grid_index(laser_x, laser_y);
        if (map_data[grid_index.first][grid_index.second] != 0) return 0.;
    }
    
    float total = 0.;
    for (const auto& laser : laser_data) 
    {
        float real_distance = laser.first;

        float ray_x = laser_x;
        float ray_y = laser_y;
        float ray_angle = particle.theta + laser.second;
        float r = RAY_STEP_SIZE;
        while (r < 1.0) 
        {
            ray_x = laser_x + r * std::cos(ray_angle);
            ray_y = laser_y + r * std::sin(ray_angle);

            // Break when out of bounds
            if (ray_x > x_max || ray_x < 0 || ray_y > y_max || ray_y < 0) break;

            // Break when wall is hit
            std::pair<int, int> grid_index = metric_to_grid_index(ray_x, ray_y);
            if (map_data[grid_index.first][grid_index.second] != 0) break;

            r += RAY_STEP_SIZE;
        }

        if (r > 1.0) {
            r = 1.0;
        } else if (r < RAY_STEP_SIZE) {
            r = RAY_STEP_SIZE;
        }
        
        total += (real_distance - r) * (real_distance - r);
        
    }
    return std::exp(-total);
}

geometry_msgs::Pose particle_to_viz_pose(const Particle& particle)
{
    geometry_msgs::Pose pose;

    pose.position.x = particle.x;
    pose.position.y = particle.y;
    pose.position.z = 0.05;

    tf2::Quaternion quaternion;
    quaternion.setRPY(0, 0, particle.theta);
    pose.orientation.x = quaternion.x();
    pose.orientation.y = quaternion.y();
    pose.orientation.z = quaternion.z();
    pose.orientation.w = quaternion.w();

    return pose;
}

green_fundamentals::Position particle_to_position(const Particle& particle)
{
    green_fundamentals::Position pos;

    pos.x = particle.x;
    pos.y = particle.y;
    pos.theta = particle.theta;

    pos.converged = converged;

    return pos;
}

void publish_particles()
{   
    // Position estimate
    float x, y, theta;
    for (const Particle& particle : particles)
    {
        x += particle.x * particle.weight;
        y += particle.y * particle.weight;
        theta += particle.theta * particle.weight;
    }

    green_fundamentals::Position position;
    position.x = x;
    position.y = y;
    position.theta = theta;
    position_pub.publish(position);

    // Visualizations
    geometry_msgs::PoseArray pose_array;
    pose_array.header.frame_id = "map";
    for (const Particle& particle : particles)
    {
        pose_array.poses.push_back(particle_to_viz_pose(particle));
    }
    particle_array_viz_pub.publish(pose_array);
}

// ############### CALLBACKS ###############
void laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    laser_data.clear();
    for (int i = 50; i < SUBSAMPLE_LASERS; i++) 
    {
        int index = i * (msg->ranges.size()-50) / SUBSAMPLE_LASERS;

        float real_distance = msg->ranges[index];
        if (real_distance != real_distance) {
            real_distance = 1.0;
        }            
        else if (real_distance < RAY_STEP_SIZE) {
            real_distance = RAY_STEP_SIZE;
        }

        float ray_angle = msg->angle_min + msg->angle_increment * index;

        laser_data.push_back({real_distance, ray_angle});
    }
    laser_received = true;
}

void sensor_callback(const create_fundamentals::SensorPacket::ConstPtr& msg)
{   
    last_left = current_left;
    last_right = current_right;
    current_left = msg->encoderLeft;
    current_right = msg->encoderRight;
    if (is_first_encoder_measurement) {
        last_left = current_left;
        last_right = current_right;

        is_first_encoder_measurement = false;
        return;
    }

    if (last_left == current_left && last_right == current_right)
    {
        sensor_received = false;
        return;
    }

    float distance_left = (current_left - last_left) * WHEEL_RADIUS;
    float distance_right = (current_right - last_right) * WHEEL_RADIUS; 
    float distance = (distance_left + distance_right) / 2;
    float angle_change = (distance_right - distance_left) / WHEEL_BASE;

    for (Particle& particle : particles)
    {
        // Motion Update
        float x_delta = distance * cos(particle.theta);
        float y_delta = distance * sin(particle.theta);

        particle.x += x_delta + normal_dist_pos(generator);
        particle.y += y_delta + normal_dist_pos(generator);
        particle.theta = normalize_angle(particle.theta + angle_change + normal_dist_theta(generator));
    }

    sensor_received = true;
}

void map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{    
    map_height = msg->info.height;
    map_width = msg->info.width;

    // Resize the map_data to match the map dimensions
    map_data.resize(map_height);
    for (int i = 0; i < map_height; ++i) 
    {
        map_data[i].resize(map_width);
    }

    // Fill the 2D array with the occupancy data
    for (int y = 0; y < map_height; ++y) 
    {
        for (int x = 0; x < map_width; ++x) 
        {
            // Calculate the index in the 1D data array
            int index = x + y * map_width;
            map_data[y][x] = msg->data[index];
        }
    }

    map_received = true;
    map_sub.shutdown();

    ros::param::get("x_max", x_max);
    ros::param::get("y_max", y_max);
}

// ############### MAIN ###############
int main(int argc, char **argv)
{
    ros::init(argc, argv, "mc_localization");
    ros::NodeHandle n;
    ROS_INFO("Starting node.");
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info)) {
        ros::console::notifyLoggerLevelsChanged();
    }

    ROS_INFO("Waiting for Occupancy Map...");
    map_sub = n.subscribe("map", 1, map_callback);
    ros::Rate loop_rate(20);
    while (!map_received)
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    ROS_INFO("Map received.");
    
    particle_array_viz_pub = n.advertise<geometry_msgs::PoseArray>("particle_array", 1);
    position_pub = n.advertise<green_fundamentals::Position>("position", 1);
    
    // Init particles
    particles.clear();
    for (int i = 0; i < NUM_PARTICLES; i++)
    {
        Particle particle = get_random_particle(1./NUM_PARTICLES);
        particles.push_back(particle);
    }

    ROS_INFO("Start Localization...");

    // Subscribers
    ros::Subscriber odo_sub = n.subscribe("sensor_packet", 1, sensor_callback);
    ros::Subscriber laser_sub = n.subscribe("scan_filtered", 1, laser_callback);

    float avg_weight, avg_weight_before = 0.;
    while(ros::ok()) {
        loop_rate.sleep();
        ros::spinOnce();

        // Only Update when robot did move enough.
        bool update = laser_received && (sensor_received || force_update);

        if (!update) 
        {
            publish_particles();
            continue;
        }

        ROS_INFO("Updating Weights");
        // Motion update already happened in callback
        // Sensor update
        float total_weight = 0.;
        for (Particle& particle : particles)
        {
            const float particle_weight = get_particle_weight(particle);
            particle.weight = particle_weight;
            total_weight += particle_weight;
            ROS_DEBUG("Particle (%f, %f) has weight: %f", particle.x, particle.y, particle.weight);
        }
        // Reset motion
        sensor_received = false;
        laser_received = false;
        force_update = false;

        // Normalize weights and compute w_slow and w_fast for adaptive sampling.
        if (total_weight > 0.)
        {
            avg_weight = total_weight / particles.size();
            if (avg_weight_before == 0) avg_weight_before = avg_weight;
            for (Particle& particle : particles)
            {
                particle.weight /= total_weight;
            }  
        } 
        else 
        {
            for (Particle& particle : particles)
            {
                particle.weight = 1.0 / particles.size();
            }
        }

        publish_particles();

        // Resample
        ROS_INFO("Resampling") ; 
        float cumulative_weights[particles.size() + 1];
        cumulative_weights[0] = 0.;
        for (int i = 0; i < particles.size(); i++)
        {
            cumulative_weights[i+1] = cumulative_weights[i] + particles[i].weight;
        }

        float w_diff = std::max(0., 1. - avg_weight / avg_weight_before);
        ROS_INFO("w_diff = %f, avg_weight = %f, avg_weight_before = %f", w_diff, avg_weight, avg_weight_before);
        std::vector<Particle> new_particles;
        for (int i = 0; i < NUM_PARTICLES; i++)
        {
            if (uniform_dist(generator) < w_diff)
            {
                Particle particle = get_random_particle(1./NUM_PARTICLES);
                new_particles.push_back(particle);
            } 
            else
            {
                float rand = uniform_dist(generator);
                int particle_index;
                for(particle_index = 0; particle_index < particles.size(); particle_index++)
                {
                    if((cumulative_weights[particle_index] <= rand) && (rand < cumulative_weights[particle_index+1])) break;
                }
                Particle particle = particles.at(particle_index);
                ROS_DEBUG("Draw Particle (%f, %f) with weight: %f", particle.x, particle.y, particle.weight);
                particle.weight = 1. / NUM_PARTICLES;
                new_particles.push_back(particle);
            }
        }

        particles = new_particles;
    }

    return 0;
}