#include "ros/ros.h"
#include <boost/bind.hpp>
#include <cmath>
#include <random>
#include "Eigen/Dense"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


#include "robot_constants.h"

#include "nav_msgs/OccupancyGrid.h"
#include "sensor_msgs/LaserScan.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "green_fundamentals/Position.h"
#include "create_fundamentals/SensorPacket.h"

#define SUBSAMPLE_LASERS 32
#define NUM_PARTICLES 50
#define RAY_STEP_SIZE 0.01

#define RESAMPLE_STD_POS 0.02
#define RESAMPLE_STD_THETA 0.005

struct Particle {
    Eigen::Vector2f position;
    float theta;
    float weight = 0.;
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

int counter_laser = 0;
int counter_sensor = 0;

sensor_msgs::LaserScan last_scan;
sensor_msgs::LaserScan current_scan;

float last_left = 0.;
float last_right = 0.;
float current_left = 0.;
float current_right = 0.;

// Relative motion since the last time particles were updated
float dx = 0.;
float dy = 0.;
float dtheta = 0.;

const Eigen::Vector2f laser_offset{0.13, 0.};

Particle particles[NUM_PARTICLES];

ros::Publisher marker_pub, markerarray_pub;

std::pair<int, int> metric_to_grid_index(float x, float y) 
{
    int gx = x / 0.01;
    int gy = y / 0.01;
    int row = std::min(std::max(gy, 0), map_height);
    int col = std::min(std::max(gx, 0), map_width);
    return {row, col};
}

void sensor_callback(const create_fundamentals::SensorPacket::ConstPtr& msg)
{
    last_left = current_left;
    last_right = current_right;
    
    current_left = msg->encoderLeft;
    current_right = msg->encoderRight;
    
    counter_sensor++;
}

void laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    last_scan = current_scan;
    current_scan = *msg;
    counter_laser++;
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

void init_particles()
{
    std::default_random_engine generator;
    std::uniform_real_distribution<float> uni_dist(0., 1.);
    for (int i = 0; i < NUM_PARTICLES; i++)
    {
        float x = uni_dist(generator) * (x_max - x_min);
        float y = uni_dist(generator) * (y_max - y_min);

        // TODO filter if particle is in grid map

        float theta = uni_dist(generator) * (2 * M_PI);
        Eigen::Vector2f pos{x, y};
        Particle particle{pos, theta};
        particles[i] = particle;
    }
}

void update_particles()
{

    float distance_left = (current_left - last_left) * WHEEL_RADIUS; // m
    float distance_right = (current_right - last_right) * WHEEL_RADIUS; // m
    float distance = (distance_left + distance_right) / 2;

    float delta_theta = (distance_right - distance_left) / WHEEL_BASE;

    for (int i = 0; i < NUM_PARTICLES; i++)
    {
        particles[i].position[0] += distance * cos(particles[i].theta + delta_theta/2);
        particles[i].position[1] += distance * sin(particles[i].theta + delta_theta/2);
        particles[i].theta += delta_theta;
    }
}

void evaluate_particles() 
{
    for (int p = 0; p < NUM_PARTICLES; p++)
    {
        // Add laser offset

        // Simulate laser measurements
        float total_delta = 0.;
        // Subsample laser scan
        for (int i = 0; i < SUBSAMPLE_LASERS; i++) 
        {
            int index = i * current_scan.ranges.size() / SUBSAMPLE_LASERS;
            float real_distance = current_scan.ranges[index];

            // Get laser angle
            float angle = current_scan.angle_min + current_scan.angle_increment * index;

            float ray_angle = particles[p].theta + angle;
            float r = RAY_STEP_SIZE;
            
            float ray_x = particles[p].position[0];
            float ray_y = particles[p].position[1];

            while (r <= 1.0) 
            {
                ray_x += r * std::cos(ray_angle);
                ray_y += r * std::sin(ray_angle);

                if (ray_x > x_max || ray_x < x_min || ray_y > y_max || ray_y < y_min) break;

                std::pair<int, int> grid_index = metric_to_grid_index(ray_x, ray_y);

                if (map_data[grid_index.second][grid_index.first] != 0) break;

                r += RAY_STEP_SIZE;
            }

            total_delta += fabs(r - real_distance);
            
        }

        particles[p].weight = std::exp(-total_delta);

    }
}

void resample_particles()
{
    Particle new_particles[NUM_PARTICLES];

    float max_weight = 0.;
    for (int i = 0; i < NUM_PARTICLES; i++)
    {
        if (particles[i].weight > max_weight)
        {
            max_weight = particles[i].weight;
        }
    }

    std::default_random_engine generator;
    std::uniform_real_distribution<float> uni_dist(0., 1.);
    std::normal_distribution<float> normal_dist(0., RESAMPLE_STD_POS);
    std::normal_distribution<float> normal_dist_theta(0., RESAMPLE_STD_THETA);

    int index = uni_dist(generator) * NUM_PARTICLES;
    double beta = 0.0;
    
    for (int i = 0; i < NUM_PARTICLES; ++i)
    {
        beta += uni_dist(generator) * 2 * max_weight;
        while (beta > particles[index].weight)
        {
            beta -= particles[index].weight;
            index = (index + 1) % NUM_PARTICLES;
        }
        new_particles[i] = particles[index];
        new_particles[i].position[0] += normal_dist(generator);
        new_particles[i].position[1] += normal_dist(generator);
        new_particles[i].theta += normal_dist_theta(generator);
    }

    std::copy(new_particles, new_particles + sizeof(Particle) * NUM_PARTICLES, particles);
    
}

Eigen::Vector2f rotate_vector(const Eigen::Vector2f& vec, float angle)
{
    float cosangle = std::cos(angle);
    float sinangle = std::sin(angle);

    Eigen::Matrix2f rotationMatrix;
    rotationMatrix << cosangle, -sinangle,
                      sinangle, cosangle;

    return rotationMatrix * vec;
}

visualization_msgs::Marker particle_to_marker(int particle_index, bool is_best)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time();
    marker.id = 0;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration(1);

    marker.pose.position.x = particles[particle_index].position[0];
    marker.pose.position.y = particles[particle_index].position[1];
    marker.pose.position.z = 0.;

    tf2::Quaternion quaternion;
    quaternion.setRPY(0, 0, particles[particle_index].theta);
    marker.pose.orientation.x = quaternion.x();
    marker.pose.orientation.y = quaternion.y();
    marker.pose.orientation.z = quaternion.z();
    marker.pose.orientation.w = quaternion.w();

    marker.scale.x = is_best ? 0.1 : 0.05;
    marker.scale.y = is_best ? 0.1 : 0.05;
    marker.scale.z = is_best ? 0.1 : 0.05;

    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = is_best ? 0. : 1.0 - particles[particle_index].weight;
    marker.color.g = is_best ? 0. : particles[particle_index].weight;
    marker.color.b = is_best ? 1. : 0.0;

    return marker;
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

    marker_pub.publish(particle_to_marker(index, true));

    visualization_msgs::MarkerArray marker_array;

    for (int i = 0; i < NUM_PARTICLES; i++)
    {
        marker_array.markers.push_back(particle_to_marker(i, false));
    }
    
    markerarray_pub.publish(marker_array);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mc_localization");
    ros::NodeHandle n;
    
    map_sub = n.subscribe("grid_map", 1, map_callback);

    ROS_INFO("Starting Node");

    init_particles();

    ros::Subscriber odo_sub = n.subscribe("sensor_packet", 1, sensor_callback);
    ros::Subscriber laser_sub = n.subscribe("scan_filtered", 1, laser_callback);

    marker_pub = n.advertise<visualization_msgs::Marker>("marker", 1);
    markerarray_pub = n.advertise<visualization_msgs::MarkerArray>("marker_array", 1);

    ros::Rate loop_rate(10);  // Hz
    while (ros::ok())
    {
        ros::spinOnce();
        if (!map_received || counter_laser < 2 || counter_sensor < 2) continue;

        // Update particle position
        update_particles();

        // Predict particle laser scan, compute error and adjust weight
        evaluate_particles();

        publish_particles();

        // resample new particles
        resample_particles();
        
        loop_rate.sleep();
    }
  
    return 0;
}