#include "ros/ros.h"
#include <boost/bind.hpp>
#include <cmath>
#include <random>
#include "Eigen/Dense"
#include <tf2/LinearMath/Quaternion.h>
#include <chrono>
#include <string>
#include "robot_constants.h"

#include "nav_msgs/OccupancyGrid.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud.h"
#include "geometry_msgs/Point32.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "create_fundamentals/SensorPacket.h"

#include "green_fundamentals/Position.h"
#include "green_fundamentals/StartLocalization.h"

#define NUM_PARTICLES 256

#define SUBSAMPLE_LASERS 24
#define RAY_STEP_SIZE 0.01

#define SPREAD_PARTICLE_PART 0.25
#define SPREAD_WEIGHT 0.1

#define STD_POS_INITIAL 0.05
#define STD_THETA_INITIAL 0.05

#define STD_POS_RESAMPLE 0.005
#define STD_THETA_RESAMPLE 0.01

#define STD_POS_UPDATE 0.02
#define STD_THETA_UPDATE 0.04

#define STD_POS_SPREAD 0.08
#define STD_THETA_SPREAD 0.16

bool active = false;

std::string message;

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

Particle particles[NUM_PARTICLES];

// Publishers

ros::Publisher pose_pub, posearray_pub, position_pub;

Particle get_random_particle(float x, float y, float theta) 
{
    std::normal_distribution<float> distribution_pos(0., STD_POS_INITIAL);
    std::normal_distribution<float> distribution_theta(0., STD_THETA_INITIAL * M_PI);

    float particle_x = x + distribution_pos(generator);
    float particle_y = y + distribution_pos(generator);

    float particle_theta = theta + distribution_theta(generator);

    Eigen::Vector2f pos{particle_x, particle_y};
    Particle particle{pos, particle_theta, 1.};

    return particle;
}

void init_particles(float x, float y, float theta)
{
    for (int i = 0; i < NUM_PARTICLES; i++)
    {
        particles[i] = get_random_particle(x, y, theta);      
    }
}

void update_particles()
{
    float distance_left = (current_left - last_left) * WHEEL_RADIUS; // m
    float distance_right = (current_right - last_right) * WHEEL_RADIUS; // m
    float distance = (distance_left + distance_right) / 2;

    float delta_theta = (distance_right - distance_left) / WHEEL_BASE;

    std::default_random_engine generator;
    std::normal_distribution<float> distribution_pos(1., STD_POS_UPDATE);
    std::normal_distribution<float> distribution_theta(1., STD_THETA_UPDATE);

    for (int i = 0; i < NUM_PARTICLES; i++)
    {   
        float x_delta = distance * cos(particles[i].theta + delta_theta/2);
        float y_delta = distance * sin(particles[i].theta + delta_theta/2);

        particles[i].position[0] += x_delta * distribution_pos(generator);
        particles[i].position[1] += y_delta * distribution_pos(generator);
        particles[i].theta += delta_theta * distribution_theta(generator);
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
    for (int p = 0; p < NUM_PARTICLES; p++)
    {
        evaluate_particle(p);
    }
}

int get_max_particle_idx() {
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

    return index;
}

void resample_particles()
{
    float max_weight = particles[get_max_particle_idx()].weight;
    int max_spreading_particles = NUM_PARTICLES * SPREAD_PARTICLE_PART;
    int num_spreading_particles = std::max(0, (int)floor(max_spreading_particles * (1 - (max_weight / SPREAD_WEIGHT))));

    message += "max weight: " +  std::to_string(max_weight) + "\n";
    message += "spreading particles: " + std::to_string(num_spreading_particles) + "\n";

    ROS_DEBUG("resampling... ");
    Particle new_particles[NUM_PARTICLES];    
    double beta = 0.0;    

    std::uniform_real_distribution<float> uni_dist(0., 1.);
    int index = uni_dist(generator) * NUM_PARTICLES;  

    std::normal_distribution<float> distribution_resample_pos(0., STD_POS_RESAMPLE);
    std::normal_distribution<float> distribtuion_resample_theta(0., STD_THETA_RESAMPLE * M_PI);    
    
    for (int i = 0; i < NUM_PARTICLES - num_spreading_particles; ++i)
    {
        beta += uni_dist(generator) * 2 * max_weight;

        while (beta > particles[index].weight)
        {
            beta -= particles[index].weight;
            index = (index + 1) % NUM_PARTICLES;
        }

        new_particles[i] = particles[index];
        new_particles[i].position[0] += distribution_resample_pos(generator);
        new_particles[i].position[1] += distribution_resample_pos(generator);
        new_particles[i].theta += distribtuion_resample_theta(generator);
    }

    std::normal_distribution<float> distribution_spread_pos(0., STD_POS_SPREAD);
    std::normal_distribution<float> distribtuion_spread_theta(0., STD_THETA_SPREAD * M_PI);

    for (int i = NUM_PARTICLES - num_spreading_particles; i < NUM_PARTICLES; ++i)
    {
        beta += uni_dist(generator) * 2 * max_weight;

        while (beta > particles[index].weight)
        {
            beta -= particles[index].weight;
            index = (index + 1) % NUM_PARTICLES;
        }

        new_particles[i] = particles[index];
        new_particles[i].position[0] += distribution_spread_pos(generator);
        new_particles[i].position[1] += distribution_spread_pos(generator);
        new_particles[i].theta += distribtuion_spread_theta(generator);
    }

    ROS_DEBUG("copying... ");
    for (int i = 0; i <  NUM_PARTICLES; i++) 
    {
        particles[i] = new_particles[i];      
    }
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

bool start_localization_callback(green_fundamentals::StartLocalization::Request  &req, green_fundamentals::StartLocalization::Response &res)
{
    if (req.activate) {
        ROS_INFO("Starting localization at (%f, %f) th: %f");

        init_particles(req.x, req.y, req.theta);

        active = true;
    }
    else {
        ROS_INFO("Stopping localization");        

        active = false;
    }

    return true;
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

    message += "Position: (" + std::to_string(position.x) + "," + std::to_string(position.y) + ") th: " + std::to_string(position.theta) + "\n";

    position_pub.publish(position);


    geometry_msgs::PoseStamped best_pose;
    best_pose.header.frame_id = "map";
    best_pose.pose = particle_to_pose(best_idx);

    pose_pub.publish(best_pose);

    geometry_msgs::PoseArray pose_array;
    pose_array.header.frame_id = "map";

    for (int i = 0; i < NUM_PARTICLES; i++)
    {
        pose_array.poses.push_back(particle_to_pose(i));
    }
    
    posearray_pub.publish(pose_array);
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "localizer");
    ros::NodeHandle n;

    ROS_INFO("Starting node.");

    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info)) {
        ros::console::notifyLoggerLevelsChanged();
    }
    
    map_sub = n.subscribe("map", 1, map_callback);
    
    ros::Rate map_loop_rate(10);
    while (!map_received)
    {
        ros::spinOnce();

        map_loop_rate.sleep();
    }

    ROS_INFO("Map received. Initializing Particles ...");
    init_particles(map_height / 2, map_width / 2, 0.);

    ros::Subscriber odo_sub = n.subscribe("sensor_packet", 1, sensor_callback);
    ros::Subscriber laser_sub = n.subscribe("scan_filtered", 1, laser_callback);

    position_pub = n.advertise<green_fundamentals::Position>("position", 1);
    pose_pub = n.advertise<geometry_msgs::PoseStamped>("best_pose", 1);
    posearray_pub = n.advertise<geometry_msgs::PoseArray>("particle_array", 1);

    ros::ServiceServer start_localization_service = n.advertiseService("start_localization", start_localization_callback);

    ROS_INFO("Waiting for StartLocalization request ..."); 

    ros::Rate loop_rate(30);

    while(ros::ok()) {

        loop_rate.sleep();

        auto t0 = std::chrono::high_resolution_clock::now();
        ros::spinOnce();

        if (!active) {
            continue;
        }
    
        if (!laser_received) {
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

        ROS_INFO("spin: %ld, eval: %ld, resa: %ld, publ: %ld", d1, d2, d3, d4);
        ROS_INFO("%s", message.c_str());
        message = "";
    }

    return 0;
}