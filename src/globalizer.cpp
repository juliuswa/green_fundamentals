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
#include "std_srvs/SetBool.h"
#include "green_fundamentals/Position.h"
#include "green_fundamentals/Pose.h"
#include "create_fundamentals/SensorPacket.h"

struct Particle {
    float x, y, theta, weight = 0.;
};
std::vector<Particle> particles;

std::vector<std::vector<int8_t>> map_data;
int map_height, map_width;
float x_max, y_max;

std::default_random_engine generator;
std::uniform_real_distribution<float> uniform_dist(0., 1.);

bool active = true;

// ############### HELPERS ###############
std::pair<int, int> metric_to_grid_index(float x, float y) 
{
    int gx = floor(x * 100);
    int gy = floor(y * 100);
    int row = std::min(std::max(gy, 0), map_height -1);
    int col = std::min(std::max(gx, 0), map_width -1);

    return {row, col};
}

Particle get_random_particle() 
{
    float x, y;
    std::pair<int, int> grid_index;
    do {
        x = uniform_dist(generator) * x_max;
        y = uniform_dist(generator) * y_max;
        grid_index = metric_to_grid_index(x, y);
    } while (map_data[grid_index.first][grid_index.second] != 0);
    
    float theta = uniform_dist(generator) * (2 * M_PI) - M_PI;

    Particle particle{x, y, theta, 0.};
    return particle;
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

float norm_angle(const float angle)
{
    return fmod(angle + 5*M_PI, 2*M_PI) - M_PI;
}

float angle_diff(float a, float b)
{
  double d1, d2;
  a = norm_angle(a);
  b = norm_angle(b);
  d1 = a-b;
  d2 = 2*M_PI - fabs(d1);
  if(d1 > 0)
    d2 *= -1.0;
  if(fabs(d1) < fabs(d2))
    return(d1);
  else
    return(d2);
}

bool has_converged()
{
    float mean_x, mean_y, mean_theta = 0.;

    for (const Particle& particle: particles)
    {
        mean_x += particle.x;
        mean_y += particle.y;
        mean_theta += norm_angle(particle.theta);
    }

    mean_x /= particles.size();
    mean_y /= particles.size();
    mean_theta /= particles.size();

    for (const Particle& particle: particles)
    {
        float d_x = mean_x - particle.x;
        float d_y = mean_y - particle.y;
        float d_theta = angle_diff(mean_theta, particle.theta);
        float dist = std::sqrt(d_x*d_x + d_y*d_y);
        if (dist > 0.4 || d_theta > M_PI/2) return false;
    }

    return true;
}

ros::Publisher particle_array_viz_pub, position_pub;
void publish_particles()
{   
    // Position estimate
    Particle best_particle = particles.at(0);
    for (const Particle& particle : particles)
    {
        if (particle.weight > best_particle.weight)
            best_particle = particle;
    }

    green_fundamentals::Position position;
    position.x = best_particle.x;
    position.y = best_particle.y;
    position.theta = best_particle.theta;
    position.converged = has_converged();
    position_pub.publish(position);

    if (position.converged) ROS_INFO("HAS CONVERGED");

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
bool map_received = false;
ros::Subscriber map_sub;
void map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{    
    map_height = msg->info.height;
    map_width = msg->info.width;
    float resolution = msg->info.resolution;
    x_max = map_width * resolution;
    y_max = map_height * resolution;

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
}

const float RESAMPLE_STD_POS = 0.01;
const float RESAMPLE_STD_THETA = M_PI/180.0;
std::normal_distribution<float> normal_dist_pos(0., RESAMPLE_STD_POS);
std::normal_distribution<float> normal_dist_theta(0., RESAMPLE_STD_THETA);
void motion_update(const float distance, const float angle_change, const bool add_random)
{
    for (Particle& particle : particles)
    {
        // Motion Update
        const float x_delta = distance * cos(particle.theta + angle_change/2);
        const float y_delta = distance * sin(particle.theta + angle_change/2);

        particle.x += x_delta;
        particle.y += y_delta;
        particle.theta += angle_change;

        if (add_random)
        {
            particle.x += normal_dist_pos(generator);
            particle.y += normal_dist_pos(generator);
            particle.theta += normal_dist_theta(generator);
        }
    }
}

float moved_distance, moved_angle, last_left, last_right, current_left, current_right = 0.;
bool is_first_encoder_measurement = true;
void sensor_callback(const create_fundamentals::SensorPacket::ConstPtr& msg)
{   
    if (!active) return;
    current_left = msg->encoderLeft;
    current_right = msg->encoderRight;
    if (is_first_encoder_measurement) {
        last_left = current_left;
        last_right = current_right;

        is_first_encoder_measurement = false;
        return;
    }

    float distance_left = (current_left - last_left) * WHEEL_RADIUS;
    float distance_right = (current_right - last_right) * WHEEL_RADIUS; 
    float distance = (distance_left + distance_right) / 2;
    float angle_change = (distance_right - distance_left) / WHEEL_BASE;

    motion_update(distance, angle_change, angle_diff(current_left, last_left) > 0.01 || angle_diff(current_right, last_right) > 0.01);

    last_left = current_left;
    last_right = current_right;

    moved_distance += distance;
    moved_angle += fabs(angle_change);
}

const int SUBSAMPLE_LASERS = 32;
const float RAY_STEP_SIZE = 0.01;
float get_particle_error(const Particle& particle, const std::vector<std::pair<float, float>>& laser_data, const bool squared_error = true) 
{
    float laser_x =  particle.x + 0.13 * std::cos(particle.theta);
    float laser_y =  particle.y + 0.13 * std::sin(particle.theta);

    {
        // Particle out of bounds
        if (particle.x > x_max || particle.x < 0 || particle.y > y_max || particle.y < 0) return INFINITY;
        // Laser out of bounds
        if (laser_x > x_max || laser_x < 0 || laser_y > y_max || laser_y < 0) return INFINITY;
        auto [particle_row, particle_col]  = metric_to_grid_index(particle.x, particle.y);
        // Particle inside wall
        if (map_data[particle_row][particle_col] != 0) return INFINITY;
        auto [laser_row, laser_col] = metric_to_grid_index(laser_x, laser_y);
        // Laser inside wall
        if (map_data[laser_row][laser_col] != 0) return INFINITY;
    }
    
    float total_error = 0;
    int iterations = 0;
    for (const auto& laser : laser_data) 
    {
        auto [real_distance, angle_offset] = laser;

        float ray_x = laser_x;
        float ray_y = laser_y;
        float ray_angle = particle.theta + angle_offset;
        float r = RAY_STEP_SIZE;
        while (r < 1.0) 
        {
            ray_x = laser_x + r * std::cos(ray_angle);
            ray_y = laser_y + r * std::sin(ray_angle);

            // Break when out of bounds
            if (ray_x > x_max || ray_x < 0 || ray_y > y_max || ray_y < 0) break;

            // Break when wall is hit
            auto [ray_row, ray_col] = metric_to_grid_index(ray_x, ray_y);
            if (map_data[ray_row][ray_col] != 0) break;

            r += RAY_STEP_SIZE;
        }

        if (r > 1.0) {
            r = 1.0;
        } else if (r < RAY_STEP_SIZE) {
            r = RAY_STEP_SIZE;
        }

        float error;
        if (squared_error) 
        {
            error = (real_distance - r) * (real_distance - r);
        }
        else
        {
            error = fabs(real_distance - r);
        }

        // TODO publish real and expected points

        total_error += error;
        iterations++;
    }

    return total_error;
}

std::vector<Particle> residual_resample()
{
    int N = particles.size();

    std::vector<Particle> new_particles;
    new_particles.reserve(N);

    for (const Particle& p : particles) 
    {
        int num_copies = (int)(p.weight * N);
        for (int i = 0; i < num_copies; i++)
        {
            Particle particle;
            particle.x = p.x + normal_dist_pos(generator);
            particle.y = p.y + normal_dist_pos(generator);
            particle.theta = p.theta + normal_dist_theta(generator);
            new_particles.push_back(particle);
        }
    }

}

bool force_update = true; // first localization without movement needed
const int num_particles = 5000;
const int num_ignore_sides = 50; // Leave out because of metal near sensor
const float weight_parameter = 0.9;
void laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{   
    if (!active) return;
    // Subsample lasers
    std::vector<std::pair<float, float>> laser_data;

    int start_index = num_ignore_sides;
    int end_index = msg->ranges.size() - num_ignore_sides;
    int valid_count = end_index - start_index;

    for (int i = 0; i < SUBSAMPLE_LASERS; i++) 
    {
        int index = start_index + floor(i * valid_count / SUBSAMPLE_LASERS);

        float real_distance = msg->ranges[index];
        if (real_distance != real_distance || real_distance > 1.) {
            real_distance = 1.0;
        }            
        else if (real_distance < RAY_STEP_SIZE) {
            real_distance = RAY_STEP_SIZE;
        }

        float ray_angle = msg->angle_min + msg->angle_increment * index;

        laser_data.push_back({real_distance, ray_angle});
    }

    // Check if enough movement
    bool update = fabs(moved_distance) > 0.05 || fabs(moved_angle) > 0.2 || force_update;

    if (!update) 
    {
        publish_particles();
        return;
    }

    force_update = false;
    moved_distance = 0.;
    moved_angle = 0.;

    ROS_INFO("UPDATING");

    // Motion update already happened in sensor callback

    // Sensor update
    float total_weight = 0.;
    for (Particle& particle : particles)
    {
        const float particle_error = get_particle_error(particle, laser_data);
        particle.weight = std::exp(-particle_error * weight_parameter);
        total_weight += particle.weight;
    }

    float sum = 0.;
    for (Particle& particle : particles)
    {
        particle.weight /= total_weight; // normalize weight
        sum += particle.weight;
    }

    // Resample
    std::vector<float> cum_sum(particles.size()+1);
    cum_sum[0] = 0;
    for (int i = 0; i < particles.size(); ++i) {
        cum_sum[i+1] = cum_sum[i] + particles[i].weight;
    } 

    std::vector<Particle> new_particles;
    while (new_particles.size() < num_particles)
    {
        float rand = uniform_dist(generator);
        int i;
        for (i = 0; i < particles.size(); i++)
        {
            if (rand <= cum_sum[i+1]) break;
        }
        
        Particle particle = particles.at(i);

        // ADD NOISE TO RESAMPLING
        particle.x += normal_dist_pos(generator);
        particle.y += normal_dist_pos(generator);
        particle.theta += normal_dist_theta(generator);

        new_particles.push_back(particle);
    }

    particles = new_particles;

    publish_particles();
    return;
}

void init_particles()
{
    particles.clear();
    for (int i = 0; i < num_particles; i++)
    {
        Particle particle = get_random_particle();
        particles.push_back(particle);
    }
}

bool activate(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
    res.success = true;
    if (req.data)
    {
        init_particles();
        active = true;
        return true;
    }
    else 
    {
        active = false;
        return true;
    }
}

// ############### MAIN ###############
int main(int argc, char **argv)
{
    ros::init(argc, argv, "mc_localization");
    ros::NodeHandle n;
    ROS_INFO("Starting node.");
    ROS_INFO("Waiting for Occupancy Map...");
    map_sub = n.subscribe("map", 1, map_callback);
    {
        ros::Rate loop_rate(5);
        while (!map_received)
        {
            ros::spinOnce();
            loop_rate.sleep();
        }
    }
    ROS_INFO("Map received.");

    particle_array_viz_pub = n.advertise<geometry_msgs::PoseArray>("particle_array", 1);
    position_pub = n.advertise<green_fundamentals::Position>("position", 1);
    ros::ServiceServer activate_service = n.advertiseService("activate_globalizer", activate);
    
    // Init particles
    init_particles();

    ROS_INFO("Start Localization...");

    // Subscribers
    ros::Subscriber odo_sub = n.subscribe("sensor_packet", 1, sensor_callback);
    ros::Subscriber laser_sub = n.subscribe("scan_filtered", 1, laser_callback);

    ros::spin();
}