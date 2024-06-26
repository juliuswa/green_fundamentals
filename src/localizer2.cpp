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

bool converged = false;

struct Particle {
    float x, y, theta, weight = 0.;
};
std::vector<Particle> particles;

ros::Subscriber map_sub;
std::vector<std::vector<int8_t>> map_data;
int map_height, map_width;
float x_max, y_max; // m
bool map_received = false;

std::default_random_engine generator;
std::uniform_real_distribution<float> uniform_dist(0., 1.);

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

float normalize_angle(float angle)
{
    angle = fmod(angle, (2 * M_PI));
    if (angle > M_PI)
        angle -= 2 * M_PI;

    return angle;
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

ros::Publisher particle_array_viz_pub, position_pub;
void publish_particles()
{   
    // Position estimate
    Particle best_particle;
    for (const Particle& particle : particles)
    {
        if (particle.weight > best_particle.weight)
            best_particle = particle;
    }

    green_fundamentals::Position position;
    position.x = best_particle.x;
    position.y = best_particle.y;
    position.theta = best_particle.theta;
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

std::vector<std::vector<bool>> initialize_bins2d(int x_size, int y_size) {
    // Create a 3D vector with the given dimensions and initial value
    std::vector<std::vector<bool>> vec2d(
        x_size, std::vector<bool>(y_size, false)
    );

    return vec2d;
}

std::vector<std::vector<std::vector<bool>>> initialize_bins3d(int x_size, int y_size, int z_size) {
    // Create a 3D vector with the given dimensions and initial value
    std::vector<std::vector<std::vector<bool>>> vec3d(
        x_size, std::vector<std::vector<bool>>(
            y_size, std::vector<bool>(
                z_size, false
            )
        )
    );

    return vec3d;
}

// ############### CALLBACKS ###############

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

const float RESAMPLE_STD_POS = 0.01;
const float RESAMPLE_STD_THETA = M_PI/180.0;
std::normal_distribution<float> normal_dist_pos(0., RESAMPLE_STD_POS);
std::normal_distribution<float> normal_dist_theta(0., RESAMPLE_STD_THETA);
void motion_update(const float distance, const float angle_change)
{
    for (Particle& particle : particles)
    {
        // Motion Update
        const float x_delta = distance * cos(particle.theta + angle_change/2);
        const float y_delta = distance * sin(particle.theta + angle_change/2);

        particle.x += x_delta + normal_dist_pos(generator);
        particle.y += y_delta + normal_dist_pos(generator);
        particle.theta = normalize_angle(particle.theta + angle_change + normal_dist_theta(generator));
    }
}

float moved_distance, moved_angle, last_left, last_right, current_left, current_right = 0.;
bool is_first_encoder_measurement = true;
void sensor_callback(const create_fundamentals::SensorPacket::ConstPtr& msg)
{   
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

    motion_update(distance, angle_change);

    last_left = current_left;
    last_right = current_right;

    moved_distance += distance;
    moved_angle += fabs(angle_change);
}

std::vector<std::pair<float, float>> laser_data;

const int SUBSAMPLE_LASERS = 32;
const float RAY_STEP_SIZE = 0.01;

float get_particle_error(const Particle& particle) 
{
    float laser_x =  particle.x + 0.13 * std::cos(particle.theta);
    float laser_y =  particle.y + 0.13 * std::sin(particle.theta);

    {
        if (particle.x > x_max || particle.x < 0 || particle.y > y_max || particle.y < 0) return INFINITY;
        if (laser_x > x_max || laser_x < 0 || laser_y > y_max || laser_y < 0) return INFINITY;
        std::pair<int, int> grid_index = metric_to_grid_index(particle.x, particle.y);
        if (map_data[grid_index.first][grid_index.second] != 0) return INFINITY;
        grid_index = metric_to_grid_index(laser_x, laser_y);
        if (map_data[grid_index.first][grid_index.second] != 0) return INFINITY;
    }
    
    float total_error = 0;
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

        float error = fabs(real_distance - r);

        // TODO publish real and expected points

        total_error += error;
    }
    return total_error;
}

bool force_update = true; // first localization without movement needed
bool use_selective_resampling = false;

float avg_short, avg_long = 0.;
const float alpha_short = 0.;
const float alpha_long = 0.;

const int resample_interval = 1;
int resample_count = 0.;

const int min_particles = 100;
const int max_particles = 5000;
const int num_particles = 1;

struct Bins {

    std::vector<std::vector<bool>> bins;

    float bin_width, bin_height;

    int num_bins_with_support = 0;

    Bins(float num_bins_on_one_axis) {
        bin_width = x_max / num_bins_on_one_axis;
        bin_height = y_max / num_bins_on_one_axis;

        bins = initialize_bins2d(num_bins_on_one_axis, num_bins_on_one_axis);
    }

    bool add_particle(float x, float y) {

        int bin_x =  (int)floor(x / bin_width);
        int bin_y =  (int)floor(y / bin_height);

        if (bins[bin_x][bin_y] == false) {
            num_bins_with_support++;
            bins[bin_x][bin_y] = true;
            return true;
        }
        else return false;
    }

    int get_new_sample_size(float upper_quantile=3, float epsilon=0.1) {
        float x = 1.0 - 2.0 / (9.0 * (num_bins_with_support - 1)) + sqrt(2.0 / (9.0 * (num_bins_with_support - 1))) * upper_quantile;
        return ceil((num_bins_with_support - 1) / (2.0 * epsilon) * x * x * x);
    }

};

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

    // Motion update already happened in callback

    // Sensor update
    float total_weight = 0.;
    for (Particle& particle : particles)
    {
        const float particle_error = get_particle_error(particle);
        particle.weight = std::exp(-particle_error * 0.9);
        ROS_INFO("PARTICLE_WEIGHT %f", particle.weight);
        total_weight += particle.weight;
    }

    if (total_weight > 0.000001)
    {
        // normalize weight
        float avg_weight = 0.;
        for (Particle& particle : particles)
        {
            avg_weight += particle.weight;
            particle.weight /= total_weight;
        }
        avg_weight /= particles.size();
        if(avg_long == 0.0)
            avg_long = avg_weight;
        else
            avg_long += alpha_long * (avg_long);
        if(avg_short == 0.0)
            avg_short = avg_weight;
        else
            avg_short += alpha_short * (avg_weight - avg_short);
    }
    else
    {
        ROS_INFO("TOTAL WEIGHT IS LOW");
        for (Particle& particle : particles)
        {
            particle.weight = 1.0 / particles.size();
        }
    }

    // Resample
    std::vector<float> cum_sum(particles.size());
    cum_sum[0] = particles[0].weight;
    for (int i = 1; i < particles.size(); ++i) {
        cum_sum[i] = cum_sum[i - 1] + particles[i].weight;
    }

    float w_diff = std::max(0., 1. - avg_short / avg_long);

    std::vector<Particle> new_particles;

    //Bins bin_grid(36);

    int num_req_particles = min_particles;
    while (new_particles.size() < num_particles)//while (new_particles.size() < num_req_particles)
    {
        Particle particle;
        if (uniform_dist(generator) < w_diff)
        {
            ROS_INFO("Sampling Random"); 
            particle = get_random_particle();
        } 
        else
        {
            float rand = uniform_dist(generator);
            int particle_index = 0;
            while (particle_index < particles.size() && cum_sum[particle_index] < rand)
            {
                particle_index++;
            }
            
            if (particle_index >= particles.size() || particle_index < 0) 
            {
                ROS_INFO("HACK");
                particle_index = particles.size() -1;
            }
            
            particle = particles.at(particle_index);

            // ADD NOISE TO RESAMPLING
            particle.x += normal_dist_pos(generator);
            particle.y += normal_dist_pos(generator);
            particle.theta = normalize_angle(particle.theta + normal_dist_theta(generator));
        }

        new_particles.push_back(particle);
        // if (bin_grid.add_particle(particle.x, particle.y))
        // {
        //     // Falls into empty bin
        //     num_req_particles = std::min(std::max(bin_grid.get_new_sample_size(), min_particles), max_particles);
        // }
    }

    // Normalize weights
    // for (Particle& particle : new_particles)
    // {
    //     particle.weight /= new_particles.size();
    // }

    particles = new_particles;

    publish_particles();
    return;
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
    for (int i = 0; i < max_particles; i++)
    {
        Particle particle = get_random_particle();
        particle.weight = 1./max_particles;
        particles.push_back(particle);
    }

    ROS_INFO("Start Localization...");

    // Subscribers
    ros::Subscriber odo_sub = n.subscribe("sensor_packet", 1, sensor_callback);
    ros::Subscriber laser_sub = n.subscribe("scan_filtered", 1, laser_callback);

    ros::spin();
}