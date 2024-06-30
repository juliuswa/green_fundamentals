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
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_srvs/SetBool.h"
#include "green_fundamentals/Position.h"
#include "green_fundamentals/Pose.h"
#include "create_fundamentals/SensorPacket.h"
#include "./localizer_base.cpp"

#define NUM_PARTICLES 5000
#define CONVERGED_NUM 100

#define SUBSAMPLE_LASERS 24
#define RAY_STEP_SIZE 0.01

#define STD_POS_RESAMPLE 0.005
#define STD_THETA_RESAMPLE 0.01

#define STD_POS_UPDATE 0.02
#define STD_THETA_UPDATE 0.04

bool force_update = true; // first localization without movement needed
const int num_ignore_sides = 50; // Leave out because of metal near sensor
const float weight_parameter = 0.5;

bool active = true;
std::string message;

// Receive Map
ros::Subscriber map_sub;
bool map_received = false;

// Map data
std::vector<std::vector<int8_t>> map_data;
int map_height, map_width;

float x_max = CELL_LENGTH * (float) MAP_WIDTH;
float y_max = CELL_LENGTH * (float) MAP_HEIGHT;

// Particles
Particle particles[NUM_PARTICLES];

// Sensor
float moved_distance = 0.;
float moved_angle = 0.;
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
std::uniform_real_distribution<float> uniform_dist(0., 1.);

// ROS
ros::Publisher pose_pub, posearray_pub, position_pub;


// ############### HELPERS ###############
// std::pair<int, int> metric_to_grid_index(float x, float y) 
// {
//     int gx = floor(x * 100);
//     int gy = floor(y * 100);
//     int row = std::min(std::max(gy, 0), map_height -1);
//     int col = std::min(std::max(gx, 0), map_width -1);

//     return {row, col};
// }

Particle get_random_particle() 
{
    float x = uniform_dist(generator) * x_max;
    float y = uniform_dist(generator) * y_max;
    
    float theta = uniform_dist(generator) * (2 * M_PI) - M_PI;

    Particle particle;
    particle.position[0] = x;
    particle.position[1] = y;
    particle.theta = theta;

    return particle;
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

bool has_converged_fast()
{
    float mean_x = 0.; 
    float mean_y = 0.;
    float mean_theta = 0.;
    
    ROS_INFO("initial mean_x: %f, mean_y: %f", mean_x, mean_y);

    for (int i = 0; i < CONVERGED_NUM; i++)
    {
        int rand = (int)(uniform_dist(generator) * NUM_PARTICLES);
        int index = std::min((int)NUM_PARTICLES-1, std::max(0, rand));

        mean_x += particles[index].position[0];
        mean_y += particles[index].position[1];
        mean_theta += norm_angle(particles[index].theta);
    }

    mean_x = mean_x / (float)CONVERGED_NUM;
    mean_y = mean_y / (float)CONVERGED_NUM;
    mean_theta = mean_theta / (float)CONVERGED_NUM;

    for (int i = 0; i < CONVERGED_NUM; i++)
    {
        int rand = (int)(uniform_dist(generator) * NUM_PARTICLES);
        int index = std::min((int)NUM_PARTICLES-1, std::max(0, rand));

        float d_x = mean_x - particles[index].position[0];
        float d_y = mean_y - particles[index].position[1];
        float d_theta = angle_diff(mean_theta, particles[index].theta);

        float dist = std::sqrt(d_x*d_x + d_y*d_y);

        if (dist > 0.4 || d_theta > M_PI/2)  {
            return false;
        }
    }

    return true;
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

void publish_particles()
{
    int best_idx = get_max_particle_idx();

    green_fundamentals::Position position;
    position.x = particles[best_idx].position[0];
    position.y = particles[best_idx].position[1];
    position.theta = particles[best_idx].theta;
    position.converged = has_converged_fast();
    position_pub.publish(position);

    if (position.converged) {
        ROS_INFO("HAS CONVERGED");
    } 

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

// ############### CALLBACKS ###############
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
    // ROS_DEBUG("sensor_callback");
    if (!active) return;

    current_left = msg->encoderLeft;
    current_right = msg->encoderRight;

    if (is_first_encoder_measurement) {
        last_left = current_left;
        last_right = current_right;

        is_first_encoder_measurement = false;
        return;
    }

    float distance_left = (current_left - last_left) * WHEEL_RADIUS; // m
    float distance_right = (current_right - last_right) * WHEEL_RADIUS; // m
    float distance = (distance_left + distance_right) / 2;
    float delta_theta = (distance_right - distance_left) / WHEEL_BASE;

    moved_distance += distance;
    moved_angle += fabs(delta_theta);

    for (int i = 0; i < NUM_PARTICLES; i++)
    {   
        float x_delta = distance * cos(particles[i].theta + delta_theta/2);
        float y_delta = distance * sin(particles[i].theta + delta_theta/2);

        particles[i].position[0] += x_delta;
        particles[i].position[1] += y_delta;
        particles[i].theta += delta_theta;
    }

    last_left = current_left;
    last_right = current_right;    
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

            if (ray_x > x_max || ray_x < 0 || ray_y > y_max || ray_y < 0)
                break;
            
            int x_i = std::min(std::max((int)(ray_x * 100.), 0), max_width_idx);
            int y_i = std::min(std::max((int)(ray_y * 100.), 0), max_height_idx);

            if (map_data[y_i][x_i] != 0)
                break;
        }

        total_delta += fabs(r - real_distance);
    }

    particles[p].weight = std::exp(-total_delta * weight_parameter);
}

void evaluate_particles() 
{
    float total_weight = 0.;

    for (int p = 0; p < NUM_PARTICLES; p++)
    {
        evaluate_particle(p);
        total_weight += particles[p].weight;
    }
    
    ROS_DEBUG("total weight: %f", total_weight);

    for (int p = 0; p < NUM_PARTICLES; p++)
    {
        particles[p].weight /= total_weight;
    }
}

bool should_resample() {
    bool update = fabs(moved_distance) > 0.01 || fabs(moved_angle) > 0.01 || force_update;

    if (!update) 
    {
        ROS_DEBUG("Not resampling. dist: %f, th: %f", moved_distance, moved_angle);
        return false;
    }

    force_update = false;
    moved_distance = 0.;
    moved_angle = 0.;

    return true;
}

void residual_resample()
{
    ROS_DEBUG("residual_resample... ");

    if (!should_resample()) {
        return;
    }
    
    Particle new_particles[NUM_PARTICLES];    
    float residuals[NUM_PARTICLES];

    std::normal_distribution<float> distribution_resample_pos(0., STD_POS_RESAMPLE);
    std::normal_distribution<float> distribtuion_resample_theta(0., STD_THETA_RESAMPLE * M_PI);    
    
    float sum = 0.;
    int particles_added = 0;

    for (int i = 0; i < NUM_PARTICLES; ++i)
    {
        int num_copies = (int)(particles[i].weight * NUM_PARTICLES);
        float residual = (particles[i].weight * NUM_PARTICLES) - num_copies;
        residuals[i] = residual;
        sum += residual;

        for (int j = 0; j < num_copies; j ++)
        {
            new_particles[particles_added] = particles[i];
            new_particles[particles_added].position[0] += distribution_resample_pos(generator);
            new_particles[particles_added].position[1] += distribution_resample_pos(generator);
            new_particles[particles_added].theta += distribtuion_resample_theta(generator);

            particles_added += 1;
        }
    }

    ROS_DEBUG("%d particles added without residual... ", particles_added);

    for (int i = 0; i < NUM_PARTICLES; i++)
    {
        residuals[i] /= sum;
    }

    float cum_sum[NUM_PARTICLES + 1];

    cum_sum[0] = 0;

    for (int i = 0; i < NUM_PARTICLES; ++i) {
        cum_sum[i+1] = cum_sum[i] + residuals[i];
    }

    for (int i = particles_added; i < NUM_PARTICLES; i++)
    {
        float rand = uniform_dist(generator);

        int index = 0;
        while (rand > cum_sum[index + 1])
        {
            index += 1;
        }

        new_particles[i] = particles[index];
        new_particles[i].position[0] += distribution_resample_pos(generator);
        new_particles[i].position[1] += distribution_resample_pos(generator);
        new_particles[i].theta += distribtuion_resample_theta(generator);

        particles_added += 1;
    }

    ROS_DEBUG("copying... ");
    for (int i = 0; i <  NUM_PARTICLES; i++) 
    {
        particles[i] = new_particles[i];      
    }
}

void laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{   
    if (!active) return;

    //ROS_DEBUG("laser_callback");
    // Subsample lasers

    laser_ranges.clear();

    for (const auto& range : msg->ranges)
    {
        laser_ranges.push_back(range);
    }

    laser_angle_min = msg->angle_min;
    laser_angle_increment = msg->angle_increment;

    laser_received = true;

    return;
}

void init_particles()
{
    for (int i = 0; i < NUM_PARTICLES; i++)
    {
        particles[i] = get_random_particle();
    }
}

bool activate(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
    res.success = true;
    if (req.data)
    {
        init_particles();

        is_first_encoder_measurement = true;
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
    ros::init(argc, argv, "globalizer");
    ros::NodeHandle n;

    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
        ros::console::notifyLoggerLevelsChanged();
    }

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

    position_pub = n.advertise<green_fundamentals::Position>("position", 1);
    pose_pub = n.advertise<geometry_msgs::PoseStamped>("best_pose", 1);
    posearray_pub = n.advertise<geometry_msgs::PoseArray>("particle_array", 1);

    ros::ServiceServer activate_service = n.advertiseService("activate_globalizer", activate);
    
    // Init particles
    init_particles();

    ROS_INFO("Start Localization...");

    // Subscribers
    ros::Subscriber odo_sub = n.subscribe("sensor_packet", 1, sensor_callback);
    ros::Subscriber laser_sub = n.subscribe("scan_filtered", 1, laser_callback);

    auto last_time = std::chrono::high_resolution_clock::now();
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
        residual_resample();

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
}