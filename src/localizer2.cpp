#include "ros/ros.h"
#include <cmath>
#include <random>
#include <tf2/LinearMath/Quaternion.h>
#include "robot_constants.h"
#include "nav_msgs/OccupancyGrid.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "green_fundamentals/StartLocalization.h"
#include "green_fundamentals/Position.h"
#include "green_fundamentals/Pose.h"
#include "create_fundamentals/SensorPacket.h"

struct Particle {
    float x, y, theta, weight;
};

const int NUM_PARTICLES = 200;
const float SPREAD_PARTICLE_PART = 0.25;
const float SPREAD_WEIGHT = 0.1;
Particle set1[NUM_PARTICLES];
Particle set2[NUM_PARTICLES];
float residuals[NUM_PARTICLES];
Particle* active_set = set1;  // Pointer to the active set
Particle* inactive_set = set2;  // Pointer to the inactive set

ros::Subscriber map_sub;
bool map_received = false;
std::vector<int8_t> map_data;
int map_height = 0, map_width = 0;
float x_max = 0., y_max = 0.;

const int SUBSAMPLE_LASERS = 32;
const float RAY_STEP_SIZE = 0.02;
const int NUM_IGNORE_SIDES = 50; // Leave out because of metal near sensor

float cum_sum[NUM_PARTICLES+1];

const float RESAMPLE_STD_POS = 0.01;
const float RESAMPLE_STD_THETA = M_PI/180.;
const float UPDATE_STD_POS = 0.01;
const float UPDATE_STD_THETA = M_PI/180.;
const float INIT_STD_POS = 0.1;
const float INIT_STD_THETA = 5 * M_PI/180.;
const float SPREAD_STD_POS = 0.05;
const float SPREAD_STD_THETA = 5 * M_PI/180.;;
std::default_random_engine generator;
std::normal_distribution<float> gaussian_pos_resample(0., RESAMPLE_STD_POS);
std::normal_distribution<float> gaussian_theta_resample(0., RESAMPLE_STD_THETA);
std::normal_distribution<float> gaussian_pos_update(0., UPDATE_STD_POS);
std::normal_distribution<float> gaussian_theta_update(0., UPDATE_STD_THETA);
std::normal_distribution<float> gaussian_pos_init(0., INIT_STD_POS);
std::normal_distribution<float> gaussian_theta_init(0., INIT_STD_THETA);
std::normal_distribution<float> gaussian_pos_spread(0., SPREAD_STD_POS);
std::normal_distribution<float> gaussian_theta_spread(0., SPREAD_STD_THETA);
std::uniform_real_distribution<float> uniform(0., 1.);

ros::Publisher particle_array_pub, best_particle_pub, position_pub;

bool active = false;

float last_left = 0.;
float last_right = 0.;
float current_left = 0.;
float current_right = 0.;
bool is_first_encoder_measurement = true;

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

inline int get_map_index(const int row, const int col) {
    return row * map_width + col;
}

bool is_cell_occupied(const float x, const float y) 
{
    int gx = floor(x * 100);
    int gy = floor(y * 100);
    int row = std::min(std::max(gy, 0), map_height -1);
    int col = std::min(std::max(gx, 0), map_width -1);

    return map_data[get_map_index(row, col)] != 0;
}

bool is_out_of_bounds(const float x, const float y)
{
    return x > x_max || x < 0 || y > y_max || y < 0;
}

Particle get_random_particle(const float x, const float y, const float theta) 
{
    float p_x, p_y;
    do {
        p_x = x + gaussian_pos_init(generator);
        p_y = y + gaussian_pos_init(generator);
    } while (is_cell_occupied(p_x, p_y));
    
    float p_theta = theta + gaussian_theta_init(generator);

    return {p_x, p_y, p_theta, 0.};
}

void init_particles(const float x, const float y, const float theta)
{
    for (int i = 0; i < NUM_PARTICLES; i++)
    {
        set1[i] = get_random_particle(x, y, theta);
    }

    for (int i = 0; i < NUM_PARTICLES; i++)
    {
        set2[i] = {0., 0., 0., 0.};
    }

    active_set = set1;
    inactive_set = set2;
}

void map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg) 
{    
    map_height = msg->info.height;
    map_width = msg->info.width;
    float resolution = msg->info.resolution;
    x_max = map_width * resolution;
    y_max = map_height * resolution;

    ROS_INFO("X_MAX=%f, Y_MAX=%f", x_max, y_max);
    ROS_INFO("MAP_HEIGHT=%d, MAP_WIDTH=%d", map_height, map_width);

    map_data = msg->data;

    map_received = true;
    map_sub.shutdown();
}

bool activate(green_fundamentals::StartLocalization::Request  &req, green_fundamentals::StartLocalization::Response &res)
{
    if (req.activate)
    {
        ROS_INFO("ACTIVATED");
        init_particles(req.x, req.y, req.theta);
        is_first_encoder_measurement = true;
        active = true;
        return true;
    }
    else 
    {
        ROS_INFO("DEACTIVATED");
        active = false;
        return true;
    }
}

void motion_update(const float distance, const float angle_change)
{
    for (int i = 0; i < NUM_PARTICLES; i++)
    {
        Particle* particle = &active_set[i];

        const float x_delta = distance * cos(particle->theta + angle_change/2);
        const float y_delta = distance * sin(particle->theta + angle_change/2);

        particle->x += x_delta + gaussian_pos_update(generator);
        particle->y += y_delta + gaussian_pos_update(generator);
        particle->theta += angle_change + gaussian_theta_update(generator);
    }
}

void sensor_callback(const create_fundamentals::SensorPacket::ConstPtr& msg)
{
    if (!active) return;

    current_left = msg->encoderLeft;
    current_right = msg->encoderRight;
    
    if (is_first_encoder_measurement) 
    {
        last_left = current_left;
        last_right = current_right;
        is_first_encoder_measurement = false;
        return;
    }

    const float distance_left = (current_left - last_left) * WHEEL_RADIUS;
    const float distance_right = (current_right - last_right) * WHEEL_RADIUS; 
    const float distance = (distance_left + distance_right) / 2;
    const float angle_change = (distance_right - distance_left) / WHEEL_BASE;

    motion_update(distance, angle_change);

    last_left = current_left;
    last_right = current_right;
}

geometry_msgs::Pose particle_to_pose(Particle *particle)
{
    geometry_msgs::Pose pose;

    pose.position.x = particle->x;
    pose.position.y = particle->y;
    pose.position.z = 0.05;

    tf2::Quaternion quaternion;
    quaternion.setRPY(0, 0, particle->theta);
    pose.orientation.x = quaternion.x();
    pose.orientation.y = quaternion.y();
    pose.orientation.z = quaternion.z();
    pose.orientation.w = quaternion.w();

    return pose;
}

bool has_converged()
{
    float mean_x = 0.; 
    float mean_y = 0.;
    float mean_theta = 0.;

    for (int i = 0; i < NUM_PARTICLES; i++)
    {
        Particle* particle = &active_set[i];

        mean_x += particle->x;
        mean_y += particle->y;
        mean_theta += norm_angle(particle->theta);
    }

    mean_x /= NUM_PARTICLES;
    mean_y /= NUM_PARTICLES;
    mean_theta /= NUM_PARTICLES;

    for (int i = 0; i < NUM_PARTICLES; i++)
    {
        Particle* particle = &active_set[i];

        const float d_x = mean_x - particle->x;
        const float d_y = mean_y - particle->y;
        const float d_theta = angle_diff(mean_theta, particle->theta);
        const float dist = std::sqrt(d_x*d_x + d_y*d_y);

        if (dist > 0.4 || d_theta > M_PI/2)  
        {
            return false;
        }
    }

    return true;
}

Particle* get_best_particle()
{
    Particle* best_particle = &active_set[0];
    
    for (int i = 0; i < NUM_PARTICLES; i++)
    {
        Particle* particle = &active_set[i];
        if (particle->weight > best_particle->weight)
        {
            best_particle = &active_set[i];
        }
    }

    return best_particle;
}

void publish_particles()
{
    Particle* best_particle = get_best_particle();

    green_fundamentals::Position position;
    position.x = best_particle->x;
    position.y = best_particle->y;
    position.theta = best_particle->theta;
    position.converged = has_converged();
    position_pub.publish(position);

    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.pose = particle_to_pose(best_particle);
    best_particle_pub.publish(pose);

    geometry_msgs::PoseArray pose_array;
    pose_array.header.frame_id = "map";
    for (int i = 0; i < NUM_PARTICLES; i++)
    {
        pose_array.poses.push_back(particle_to_pose(&active_set[i]));
    }
    particle_array_pub.publish(pose_array);
}

float get_particle_error(const Particle *particle, const sensor_msgs::LaserScan::ConstPtr& msg) 
{
    const float laser_x =  particle->x + 0.13 * std::cos(particle->theta);
    const float laser_y =  particle->y + 0.13 * std::sin(particle->theta);

    if (is_out_of_bounds(particle->x, particle->y)) return INFINITY;
    if (is_out_of_bounds(laser_x, laser_y)) return INFINITY;
    if (is_cell_occupied(particle->x, particle->y)) return INFINITY;
    if (is_cell_occupied(laser_x, laser_y)) return INFINITY;

    const int start_index = NUM_IGNORE_SIDES;
    const int end_index = msg->ranges.size() - NUM_IGNORE_SIDES;
    const int valid_count = end_index - start_index;

    float total_error = 0;
    int index;
    float real_distance;
    float angle_offset;
    for (int i = 0; i < SUBSAMPLE_LASERS; i++) 
    {
        index = start_index + floor(i * valid_count / SUBSAMPLE_LASERS);
        real_distance = msg->ranges[index];
        if (real_distance != real_distance || real_distance > 1.) 
        {
            real_distance = 1.0;
        }            
        else if (real_distance < RAY_STEP_SIZE) 
        {
            real_distance = RAY_STEP_SIZE;
        }

        angle_offset = msg->angle_min + msg->angle_increment * index;

        float ray_x = laser_x;
        float ray_y = laser_y;
        const float ray_angle = particle->theta + angle_offset;
        const float ray_x_increment = RAY_STEP_SIZE * std::cos(ray_angle);
        const float ray_y_increment = RAY_STEP_SIZE * std::sin(ray_angle);
        float r = RAY_STEP_SIZE;
        while (r < 1.0) 
        {
            ray_x += ray_x_increment;
            ray_y += ray_y_increment;

            if (is_out_of_bounds(ray_x, ray_y)) break;
            if (is_cell_occupied(ray_x, ray_y)) break;

            r += RAY_STEP_SIZE;
        }

        if (r > 1.0) {
            r = 1.0;
        } else if (r < RAY_STEP_SIZE) {
            r = RAY_STEP_SIZE;
        }

        total_error += (real_distance - r) * (real_distance - r);
    }

    return total_error;
}

void resample()
{
    Particle* best_particle = get_best_particle();

    const int max_spreading_particles = NUM_PARTICLES * SPREAD_PARTICLE_PART;
    const int num_spreading_particles = std::max(0, (int)floor(max_spreading_particles * (1 - best_particle->weight / SPREAD_WEIGHT)));
    
    double beta = 0.;
    int index = uniform(generator) * NUM_PARTICLES;

    for (int i = 0; i < NUM_PARTICLES - num_spreading_particles; i++)
    {
        beta += uniform(generator) * 2 * best_particle->weight;

        while (beta > active_set[index].weight)
        {
            beta -= active_set[index].weight;
            index = (index + 1) % NUM_PARTICLES;
        }

        Particle* particle = &active_set[index];
        inactive_set[i] = {
            particle->x + gaussian_pos_resample(generator),
            particle->y + gaussian_pos_resample(generator),
            particle->theta + gaussian_theta_resample(generator),
            0.
        };
    }

    for (int i = NUM_PARTICLES - num_spreading_particles; i < NUM_PARTICLES; i++)
    {
        beta += uniform(generator) * 2 * best_particle->weight;

        while (beta > active_set[index].weight)
        {
            beta -= active_set[index].weight;
            index = (index + 1) % NUM_PARTICLES;
        }

        Particle* particle = &active_set[index];
        inactive_set[i] = {
            particle->x + gaussian_pos_spread(generator),
            particle->y + gaussian_pos_spread(generator),
            particle->theta + gaussian_theta_spread(generator),
            0.
        };
    }
}

void laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{   
    if (!active) return;
    auto t0 = std::chrono::high_resolution_clock::now();

    ROS_INFO("UPDATING");

    // Motion update already happened in sensor callback

    // Sensor update
    float total_weight = 0.;
    for (int i = 0; i < NUM_PARTICLES; i++)
    {
        Particle* particle = &active_set[i];
        const float particle_error = get_particle_error(particle, msg);
        const float weight = std::exp(-particle_error);
        particle->weight = (particle->weight + weight) / 2;
        total_weight += weight;
    }

    for (int i = 0; i < NUM_PARTICLES; i++)
    {
        Particle* particle = &active_set[i];
        particle->weight /= total_weight;
    }

    // Resample
    resample();

    publish_particles();

    std::swap(active_set, inactive_set);

    auto t4= std::chrono::high_resolution_clock::now();
    auto d1 = std::chrono::duration_cast<std::chrono::milliseconds>(t4 - t0).count();

    ROS_INFO("laser_callback complete: %ld", d1);

    return;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "localizer");
    ros::NodeHandle n;
    map_sub = n.subscribe("map", 1, map_callback);
    {
        ros::Rate loop_rate(5);
        while (!map_received)
        {
            ros::spinOnce();
            loop_rate.sleep();
        }
    }

    particle_array_pub = n.advertise<geometry_msgs::PoseArray>("particle_array", 1);
    best_particle_pub = n.advertise<geometry_msgs::PoseStamped>("best_particle", 1);
    position_pub = n.advertise<green_fundamentals::Position>("position", 1);
    ros::ServiceServer activate_service = n.advertiseService("activate_localizer", activate);

    ROS_INFO("Start Localization...");

    // Subscribers
    ros::Subscriber odo_sub = n.subscribe("sensor_packet", 1, sensor_callback);
    ros::Subscriber laser_sub = n.subscribe("scan_filtered", 1, laser_callback);

    ros::spin();
}