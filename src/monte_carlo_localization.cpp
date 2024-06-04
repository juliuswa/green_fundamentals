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
#define NUM_PARTICLES 400
#define NUM_RANDOM_PARTICLES 5
#define RAY_STEP_SIZE 0.01
#define RESAMPLE_STD_POS 0.02
#define RESAMPLE_STD_THETA 0.04
#define DISTANCE_THRESHOLD 0.01
#define THETA_THRESHOLD 0.02

struct Particle {
    float x, y, theta;
};

// Map
ros::Subscriber map_sub;
bool map_received = false;

std::vector<std::vector<int8_t>> map_data;
int map_height, map_width;

float x_max, y_max; // m

// Encoder values
float last_left = 0.;
float last_right = 0.;

float current_left = 0.;
float current_right = 0.;

bool is_first_encoder_measurement = true;

// Relative motion since the last time particles were updated
float relative_distance = 0.;
float relative_theta = 0.;

// Random generators
std::default_random_engine generator;

std::uniform_real_distribution<float> uniform_dist(0., 1.);
std::normal_distribution<float> normal_dist_pos(0., RESAMPLE_STD_POS);
std::normal_distribution<float> normal_dist_theta(0., RESAMPLE_STD_THETA);

// Particles
Particle particles[NUM_PARTICLES];

// Publishers
ros::Publisher pose_pub, posearray_pub, actual_ray_pub, expected_ray_pub;

/*
    Compute Map Indexes from metric Coordinates.
*/
std::pair<int, int> metric_to_grid_index(float x, float y) 
{
    int gx = floor(x * 100);
    int gy = floor(y * 100);
    int row = std::min(std::max(gy, 0), map_height -1);
    int col = std::min(std::max(gx, 0), map_width -1);

    return {row, col};
}

/*
    Generate random Particle that is in a free cell.
*/
Particle get_random_particle() 
{
    float x, y;
    std::pair<int, int> grid_index;
    do {
        x = uniform_dist(generator) * x_max;
        y = uniform_dist(generator) * y_max;
        grid_index = metric_to_grid_index(x, y);
    } while (map_data[grid_index.first][grid_index.second] != 0);
    
    float theta = uniform_dist(generator) * (2 * M_PI);

    Particle particle{x, y, theta};
    return particle;
}

int draw_particle_from_weight(float weights[])
{

}

/*
    Perform Ray Marching to get expected ranges and compute error to actual ranges.
*/
float get_particle_error(const sensor_msgs::LaserScan::ConstPtr& msg, const Particle& particle) 
{
    float total_delta = 0.;

    float laser_x =  particle.x + 0.13 * std::cos(particle.theta);
    float laser_y =  particle.y + 0.13 * std::sin(particle.theta);

    for (int i = 0; i < SUBSAMPLE_LASERS; i++) 
    {
        int index = i * msg->ranges.size() / SUBSAMPLE_LASERS;
        
        float real_distance = msg->ranges[index];
        if (real_distance != real_distance) {
            real_distance = 1.0;
        }            
        else if (real_distance < RAY_STEP_SIZE) {
            real_distance = RAY_STEP_SIZE;
        }

        float ray_x = laser_x;
        float ray_y = laser_y;
        float ray_angle = particle.theta + (msg->angle_min + msg->angle_increment * index);
        float r = 0.;
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

        total_delta += fabs(r - real_distance);            
    }

    return total_delta;
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

/*
    Process new Laser Scan and perform localization.
*/
void laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    float weights[NUM_PARTICLES];

    for (int i = 0; i < NUM_PARTICLES; i++)
    {   
        /*
            Motion Update:
            Move the particle by the distance the robot traveled and apply some noise.
        */

        if (relative_distance > DISTANCE_THRESHOLD || relative_theta > THETA_THRESHOLD)
        {
            // Only Move when the movement was big enough.
            // Just not move the particle or dont localize at all?
            float x_delta = relative_distance * cos(particles[i].theta + relative_theta/2);
            float y_delta = relative_distance * sin(particles[i].theta + relative_theta/2);

            particles[i].x += x_delta + normal_dist_pos(generator);
            particles[i].y += y_delta + normal_dist_pos(generator);
            particles[i].theta += relative_theta + normal_dist_theta(generator);
        }

        /*
            Sensor Update:
            Calculate the weight of the particle given the laser measurement by ray marching.
        */
        float error = get_particle_error(msg, particles[i]);
        weights[i] = std::exp(-error);
    }

    // Reset distance traveled by robot
    relative_distance = 0.;
    relative_theta = 0.;

    /*
        Resample:
        Draw new particles from the current particles proportional to the weight.
    */
    Particle new_particles[NUM_PARTICLES];
    for (int i = 0; i < NUM_PARTICLES; i++)
    {
        int particle_index = draw_particle_from_weight(weights);
        new_particles[i] = particles[particle_index];
    }

    /*
        Copy particles
    */
    for (int i = 0; i < NUM_PARTICLES; i++)
    {
        particles[i] = new_particles[i];
    }

    publish_particles();

    resample_particles();
}

/*
    Accumulate the relative motion of the robot since the last laser measurement.
*/
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
    }
    
    float distance_left = (current_left - last_left) * WHEEL_RADIUS;
    float distance_right = (current_right - last_right) * WHEEL_RADIUS;
    float distance = (distance_left + distance_right) / 2;
    float delta_theta = (distance_right - distance_left) / WHEEL_BASE;

    relative_distance += distance;
    relative_theta += delta_theta;
}

/*
    Receive Occupancy Map and then unsubscribe from the topic.
*/
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

    x_max = map_width / 100;
    y_max = map_height / 100;

    map_received = true;
    map_sub.shutdown();
}

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
    ros::Rate loop_rate(10);
    while (!map_received)
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    ROS_INFO("Map received. Starting localization...");
    // Subscribers
    ros::Subscriber odo_sub = n.subscribe("sensor_packet", 1, sensor_callback);
    ros::Subscriber laser_sub = n.subscribe("scan_filtered", 1, laser_callback);

    // Publishers
    pose_pub = n.advertise<geometry_msgs::Pose>("pose", 1);
    posearray_pub = n.advertise<geometry_msgs::PoseArray>("pose_array", 1);
    
    // Debug Pubs
    //actual_ray_pub = n.advertise<sensor_msgs::PointCloud>("actual_ray", 1);
    //expected_ray_pub = n.advertise<sensor_msgs::PointCloud>("expected_ray", 1);
    
    // Initialize particles
    for (int i = 0; i < NUM_PARTICLES; i++)
    {
        particles[i] = get_random_particle();
    }

    ros::spin();

    return 0;
}