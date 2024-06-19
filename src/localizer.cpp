#include "ros/ros.h"
#include <boost/bind.hpp>
#include <cmath>
#include <random>
#include "Eigen/Dense"
#include <tf2/LinearMath/Quaternion.h>
#include <chrono>

#include "robot_constants.h"

#include "nav_msgs/OccupancyGrid.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Point32.h"
#include "sensor_msgs/PointCloud.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseArray.h"
#include "green_fundamentals/Position.h"
#include "create_fundamentals/SensorPacket.h"

#define SUBSAMPLE_LASERS 24
#define RAY_STEP_SIZE 0.01

#define PARTICLES_PER_BIN 128
#define NUM_BINS 1296
#define FILLED_BIN_THRESHOLD 8

#define SPREAD_PARTICLE_PART 0.2
#define RANDOM_PARTICLE_PART 0.05

#define SPREAD_WEIGHT 0.2
#define RANDOM_WEIGHT 0.05

#define RESAMPLE_STD_POS 0.04
#define RESAMPLE_STD_THETA 0.08

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

float x_min = 0.0; // m
float x_max = CELL_LENGTH * (float) MAP_WIDTH;
float y_min = 0.0;
float y_max = CELL_LENGTH * (float) MAP_HEIGHT;

// Sensor

float last_left = 0.;
float last_right = 0.;

float current_left = 0.;
float current_right = 0.;

bool is_first_encoder_measurement = true;

// Laser

std::vector<double> laser_ranges;
float laser_angle_min;
float laser_angle_increment;

bool laser_received = false;

// Random

std::default_random_engine generator;

// Particles

Particle particles[PARTICLES_PER_BIN * NUM_BINS];

int sample_size = PARTICLES_PER_BIN;

int bins[NUM_BINS];
int bin_division = floor(std::sqrt(NUM_BINS));
float bin_x = (x_max - x_min) / bin_division;
float bin_y = (y_max - y_min) / bin_division;

// Publishers

ros::Publisher pose_pub, posearray_pub, position_pub;

int get_bin_for_position(int i) {
    return std::min((int)(floor(particles[i].position[0] / bin_x) + bin_division * floor(particles[i].position[1] / bin_y)), NUM_BINS - 1);
}

void set_particle(Particle particle, int index) {
    particles[index] = particle;
    int bin = get_bin_for_position(index);
    bins[bin] += 1;
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
    for (int i = 0; i < sample_size; i++)
    {
        set_particle(get_random_particle(), i);        
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

    for (int i = 0; i < sample_size; i++)
    {   
        float x_delta = distance * cos(particles[i].theta + delta_theta/2);
        float y_delta = distance * sin(particles[i].theta + delta_theta/2);

        particles[i].position[0] += x_delta * normal_dist_pos(generator);
        particles[i].position[1] += y_delta * normal_dist_pos(generator);
        particles[i].theta += delta_theta * normal_dist_theta(generator);
    }
}

void evaluate_particle(int p)
{
    int max_width_idx = map_width -1;
    int max_height_idx = map_height -1;

    float total_delta = 0.;

    float laser_x = particles[p].position[0] + LASER_OFFSET * std::cos(particles[p].theta);
    float laser_y = particles[p].position[1] + LASER_OFFSET * std::sin(particles[p].theta);

    for (int i = 0; i < SUBSAMPLE_LASERS; i++)
    {
        int index = i * laser_ranges.size() / SUBSAMPLE_LASERS;

        float real_distance = laser_ranges[index];

        if (real_distance != real_distance)
        {
            real_distance = 1.0;
        }

        // Get laser angle
        float angle = laser_angle_min + laser_angle_increment * index;
        float ray_angle = particles[p].theta + angle;
        
        float ray_x_increment = RAY_STEP_SIZE * std::cos(ray_angle);
        float ray_y_increment = RAY_STEP_SIZE * std::sin(ray_angle);

        float ray_x = laser_x;
        float ray_y = laser_y;
        // ROS_DEBUG("ray casting");

        float r = 0.0;
        for (int  i= 0; i < 100; i++)
        {
            r += RAY_STEP_SIZE;
            ray_x += ray_x_increment;
            ray_y += ray_y_increment;

            if (ray_x > x_max || ray_x < x_min || ray_y > y_max || ray_y < y_min)
                break;
            
            int x_i = std::min(std::max((int)(ray_x * 100.), 0), max_width_idx);
            int y_i = std::min(std::max((int)(ray_y * 100.), 0), max_height_idx);

            if (map_data[y_i][x_i] != 0)
                break;
        }

        total_delta += fabs(r - real_distance);
    }

    particles[p].weight = std::exp(-total_delta);
}

void evaluate_particles() 
{
    for (int p = 0; p < sample_size; p++)
    {
        evaluate_particle(p);
    }
}

int get_max_particle_idx() {
    float max_weight = 0.;
    int index = -1;

    for (int i = 0; i < sample_size; i++)
    {
        if (particles[i].weight > max_weight)
        {
            max_weight = particles[i].weight;
            index = i;
        }
    }

    return index;
}

int calculate_sample_size() {
    int filled_bin_count = 0;

    for (int i = 0; i < NUM_BINS; i++) {
        if (bins[i] > FILLED_BIN_THRESHOLD) {
            filled_bin_count += 1;
        }            
    }

    return std::max(1, filled_bin_count) * PARTICLES_PER_BIN;
}

void reset_bins() {
    for (int i = 0; i < NUM_BINS; i++) {
        bins[i] = 0;
    }
}

void resample_particles()
{
    float max_weight = particles[get_max_particle_idx()].weight;

    std::uniform_real_distribution<float> uni_dist(0., 1.);
    std::normal_distribution<float> normal_dist_pos(0., RESAMPLE_STD_POS / 8);
    std::normal_distribution<float> normal_dist_theta(0., RESAMPLE_STD_THETA / 8);

    int new_sample_size = calculate_sample_size();

    int max_spreading_particles = new_sample_size * SPREAD_PARTICLE_PART;
    int num_spreading_particles = std::max(0, (int)floor(max_spreading_particles * (1 - (max_weight / SPREAD_WEIGHT))));

    int max_random_particles = new_sample_size * RANDOM_PARTICLE_PART;
    int num_random_particles = std::max(0, (int)floor(max_random_particles * (1 - (max_weight / RANDOM_WEIGHT))));

    ROS_DEBUG("max weight: %f, sample size: %d, random particles: %d", 
        max_weight, new_sample_size, num_random_particles);

    ROS_DEBUG("resampling... ");
    Particle new_particles[new_sample_size];    
    
    int index = uni_dist(generator) * sample_size;    
    double beta = 0.0;    
    
    for (int i = 0; i < new_sample_size - num_spreading_particles - num_random_particles; ++i)
    {
        beta += uni_dist(generator) * 2 * max_weight;

        while (beta > particles[index].weight)
        {
            beta -= particles[index].weight;
            index = (index + 1) % sample_size;
        }

        new_particles[i] = particles[index];
        new_particles[i].position[0] += normal_dist_pos(generator);
        new_particles[i].position[1] += normal_dist_pos(generator);
        new_particles[i].theta += normal_dist_theta(generator);
    }

    std::normal_distribution<float> spread_dist_pos(0., RESAMPLE_STD_POS *2 );
    std::normal_distribution<float> spread_dist_theta(0., RESAMPLE_STD_THETA * 2);

    for (int i = new_sample_size - num_spreading_particles - num_random_particles; 
             i < new_sample_size - num_random_particles; ++i)
    {
        beta += uni_dist(generator) * 2 * max_weight;

        while (beta > particles[index].weight)
        {
            beta -= particles[index].weight;
            index = (index + 1) % sample_size;
        }

        new_particles[i] = particles[index];
        new_particles[i].position[0] += spread_dist_pos(generator);
        new_particles[i].position[1] += spread_dist_pos(generator);
        new_particles[i].theta += spread_dist_theta(generator);
    }

    for (int i = new_sample_size - num_random_particles; i < new_sample_size; ++i)
    {
        new_particles[i] = get_random_particle();
    }

    ROS_DEBUG("copying... ");
  
    reset_bins();
    for (int i = 0; i < new_sample_size; i++) 
    {
        set_particle(new_particles[i], i);
    }
    
    sample_size = new_sample_size;
}

void map_callback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
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

void sensor_callback(const create_fundamentals::SensorPacket::ConstPtr& msg)
{    
    ROS_DEBUG("sensor_callback");
    current_left = msg->encoderLeft;
    current_right = msg->encoderRight;

    if (is_first_encoder_measurement) {
        last_left = current_left;
        last_right = current_right;

        is_first_encoder_measurement = false;
        return;
    }

    ROS_DEBUG("update_particles");
    update_particles();

    last_left = current_left;
    last_right = current_right;
}

void laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{    
    ROS_DEBUG("laser_callback");
    laser_ranges.clear();

    for (const auto& range : msg->ranges)
    {
        laser_ranges.push_back(range);
    }

    laser_angle_min = msg->angle_min;
    laser_angle_increment = msg->angle_increment;

    laser_received = true;
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
    int best_idx = get_max_particle_idx();

    green_fundamentals::Position position;
    position.x = particles[best_idx].position[0];
    position.y = particles[best_idx].position[1];
    position.theta = particles[best_idx].theta;
    ROS_DEBUG("Publishing position: (%f, %f) th: %f", position.x, position.y, position.theta);
    position_pub.publish(position);

    pose_pub.publish(particle_to_pose(best_idx));

    geometry_msgs::PoseArray pose_array;
    pose_array.header.frame_id = "map";

    for (int i = 0; i < sample_size; i++)
    {
        pose_array.poses.push_back(particle_to_pose(i));
    }
    
    posearray_pub.publish(pose_array);
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
    
    ros::Rate map_loop_rate(10);
    while (!map_received)
    {
        ros::spinOnce();

        map_loop_rate.sleep();
    }

    ROS_INFO("Map received.");
    
    init_particles();

    ROS_INFO("Particles initialized.");

    ros::Subscriber odo_sub = n.subscribe("sensor_packet", 1, sensor_callback);
    ros::Subscriber laser_sub = n.subscribe("scan_filtered", 1, laser_callback);

    position_pub = n.advertise<green_fundamentals::Position>("position", 1);
    pose_pub = n.advertise<geometry_msgs::Pose>("best_pose", 1);
    posearray_pub = n.advertise<geometry_msgs::PoseArray>("pose_array", 1);

    ros::Rate loop_rate(30);
    int it = 0;
    while(ros::ok()) {

        auto t0 = std::chrono::high_resolution_clock::now();
        
        ros::spinOnce();

        if (!laser_received) {
            loop_rate.sleep();
            continue;
        }
        
        auto t1 = std::chrono::high_resolution_clock::now();

        ROS_DEBUG("evaluate_particles");
        evaluate_particles();

        auto t2= std::chrono::high_resolution_clock::now();
        
        ROS_DEBUG("resample_particles");
        resample_particles();

        auto t3= std::chrono::high_resolution_clock::now();

        ROS_DEBUG("publish_particles");
        publish_particles();

        auto t4= std::chrono::high_resolution_clock::now();

        auto d1 = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();
        auto d2 = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
        auto d3 = std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t2).count();
        auto d4 = std::chrono::duration_cast<std::chrono::milliseconds>(t4 - t3).count();

        ROS_DEBUG("spin: %ld, eval: %ld, resa: %ld, publ: %ld", d1, d2, d3, d4);

        loop_rate.sleep();
        it++;
    }

    return 0;
}