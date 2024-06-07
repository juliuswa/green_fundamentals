#include "ros/ros.h"
#include "std_srvs/Empty.h"
#include "nav_msgs/OccupancyGrid.h"
#include <deque>

#include "green_fundamentals/Position.h"
#include "green_fundamentals/Pose.h"
#include "green_fundamentals/DriveTo.h"
#include "green_fundamentals/ExecutePlan.h"

enum State {
    INIT,
    LOCALIZE,
    IDLE,
    EXECUTE_PLAN
};

State state = State::INIT;

enum Orientation {
    LEFT,
    RIGHT,
    UP,
    DOWN
};

struct Position {
    float x, y, theta;
    int row, col;
    Orientation orientation = Orientation::RIGHT;
    bool converged = false;
};

struct Target {
    float x, y, theta;
    bool should_rotate = false;
    bool must_be_reached = false;
};

Position my_position{0., 0., 0., 0, 0};
Target current_target{0., 0., 0.};
std::deque<Target> target_list;

// Map
ros::Subscriber map_sub;
std::vector<std::vector<int8_t>> map_data;
int map_height, map_width, grid_rows, grid_cols, wall_length_pixels, corner_pixels, wall_length;
bool map_received = false;
std::vector<std::vector<std::pair<float, float>>> cell_centers;

// Publishers
ros::Publisher pose_pub;

// Clients
ros::ServiceClient start_localize_client, mover_set_idle_client, mover_set_wander_client, mover_drive_to_client;

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

    map_received = true;
    map_sub.shutdown();

    int wall_length_pixels, corner_pixels;
    float pixel_size;

    ros::param::get("grid_num_rows", grid_rows);
    ros::param::get("grid_num_cols", grid_cols);
    ros::param::get("wall_length_pixels", wall_length_pixels);
    ros::param::get("corner_pixels", corner_pixels);
    ros::param::get("pixel_size", pixel_size);

    wall_length = (wall_length_pixels + corner_pixels) * pixel_size;

    // compute the cell centers
    cell_centers.resize(grid_rows);
    for (int i = 0; i < grid_rows; ++i) 
    {
        cell_centers[i].resize(grid_cols);
    }

    // TODO add cell information in a Cell struct so that we can check when executing plan if we will run into a wall.
    for (int row = 0; row < grid_rows; row++)
    {
        for (int col = 0; col < grid_cols; col++)
        {
            float y = map_height - (row * (corner_pixels + wall_length_pixels)) - (corner_pixels + wall_length_pixels / 2) * pixel_size;
            float x = (col * (corner_pixels + wall_length_pixels)) + (corner_pixels + wall_length_pixels / 2) * pixel_size;
            std::pair<float, float> corner{x, y};
            cell_centers[row][col] = corner;
        }
    }
}

void localization_callback(const green_fundamentals::Position::ConstPtr& msg)
{
    my_position.x = msg->x;
    my_position.y = msg->y;
    my_position.theta = msg->theta;
    my_position.converged = msg->converged;
    my_position.row = floor(my_position.y / wall_length);
    my_position.col = floor(my_position.x / wall_length);

    if (my_position.theta <= M_PI/4 && my_position.theta > -M_PI/4)
    {
        my_position.orientation = Orientation::RIGHT;
    }
    else if (my_position.theta > M_PI/4 && my_position.theta < 3*M_PI/4)
    {
        my_position.orientation = Orientation::UP;
    }
    else if (my_position.theta <= -3*M_PI/4 || my_position.theta >= 3*M_PI/4)
    {
        my_position.orientation = Orientation::LEFT;
    }
    else if (my_position.theta > -3*M_PI/4 && my_position.theta <= -M_PI/4)
    {
        my_position.orientation = Orientation::DOWN;
    }

    if (my_position.converged)
    {
        green_fundamentals::Pose pose;

        pose.row = my_position.row;
        pose.column = my_position.col;
        switch (my_position.orientation)
        {
        case Orientation::UP:
            pose.orientation = green_fundamentals::Pose::UP;
            break;
        case Orientation::DOWN:
            pose.orientation = green_fundamentals::Pose::DOWN;
            break;
        case Orientation::LEFT:
            pose.orientation = green_fundamentals::Pose::LEFT;
            break;
        case Orientation::RIGHT:
            pose.orientation = green_fundamentals::Pose::RIGHT;
            break;
        }

        pose_pub.publish(pose);
    }
}

std::pair<float, float> get_current_cell_center()
{
    return cell_centers[my_position.row][my_position.col];
}

void set_target() {
    green_fundamentals::DriveTo drive_to_msg;

    drive_to_msg.request.x_current = my_position.x;
    drive_to_msg.request.y_current = my_position.y;
    drive_to_msg.request.theta_current = my_position.theta;
    drive_to_msg.request.x_target = current_target.x;
    drive_to_msg.request.y_target = current_target.y;
    drive_to_msg.request.theta_target = current_target.theta;
    drive_to_msg.request.rotate = current_target.should_rotate;

    if (!mover_drive_to_client.call(drive_to_msg))
    {
        ROS_INFO("failed to call driver_service");
    }
}

bool current_target_reached()
{   
    if (target_list.size() == 0) {
        return true;
    }

    float pos_epsilon = target_list[0].must_be_reached ? POS_EPSILON : SOFT_EPSILON;

    bool distance_diff = fabs(my_position.x - target_list[0].x) < pos_epsilon && fabs(my_position.y - target_list[0].y) < pos_epsilon;
    bool angle_diff = fabs(my_position.theta - target_list[0].theta) < THETA_EPSILON;

    if (current_target.should_rotate) return distance_diff && angle_diff;
    
    return distance_diff;
}

bool set_execute_plan_callback(green_fundamentals::ExecutePlan::Request  &req, green_fundamentals::ExecutePlan::Response &res)
{
    target_list.clear();

    int row = my_position.row;
    int col = my_position.col;

    for (int i = 0; i < req.plan.size(); i++)
    {
        Target new_target;

        switch (req.plan[i])
        {
        case  0: // RIGHT
            col++;
            break;
        case  1: // UP
            row++;
            break;
        case  2: // LEFT
            col--;
            break;
        case  3: // DOWN
            row--;
            break;
        }

        if (row > grid_rows || row < 0 || col > grid_cols || col < 0)
        {
            target_list.clear();
            res.success = false;
            state = State::IDLE;
            return false;
        }

        std::pair<float, float> center = cell_centers[row][col];
        new_target.x = center.first;
        new_target.y = center.second;
        target_list.push_back(new_target);
    }

    state = State::EXECUTE_PLAN;
    res.success = true;
    return true;
}

void execute_plan()
{
    if (current_target_reached())
    {
        target_list.pop_front();
    }

    if (target_list.size() == 0)  {
        state = State::IDLE;
        return;
    }

    
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "local_planner");
    ros::NodeHandle n;
    ros::Rate loop_rate(15);

    ROS_INFO("Starting node.");
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info)) {
        ros::console::notifyLoggerLevelsChanged();
    }

    ROS_INFO("Initializing...");

    ROS_INFO("Waiting for Occupancy Map...");
    map_sub = n.subscribe("map", 1, map_callback);
    while (!map_received)
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    ROS_INFO("Occupancy Map received. Starting localization phase...");
    state = State::LOCALIZE;

    // Subscribers
    ros::Subscriber sensor_sub = n.subscribe("position", 1, localization_callback);

    // Publishers
    pose_pub = n.advertise<green_fundamentals::Pose>("pose", 1);

    bool localizer_available = ros::service::waitForService("localizer_start_localize", ros::Duration(10.0));
    bool mover_available = ros::service::waitForService("mover_set_idle", ros::Duration(10.0));

    if (!localizer_available || !mover_available)
    {
        ROS_ERROR("A service is not available");
        return 1;
    }

    // Services
    start_localize_client = n.serviceClient<std_srvs::Empty>("localizer_start_localize");
    mover_set_idle_client = n.serviceClient<std_srvs::Empty>("mover_set_idle");
    mover_set_wander_client = n.serviceClient<std_srvs::Empty>("mover_set_wander");
    mover_drive_to_client = n.serviceClient<green_fundamentals::DriveTo>("mover_set_drive_to");

    std_srvs::Empty empty_srv;
    start_localize_client.call(empty_srv);
    mover_set_wander_client.call(empty_srv);

    while (!my_position.converged)
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    // We converged, now align
    std::pair<float, float> target = get_current_cell_center(); // x, y

    current_target.x = target.first;
    current_target.y = target.second;
    current_target.theta = M_PI/2;
    current_target.should_rotate = true;

    set_target();

    while (!current_target_reached())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    // We are aligned, now we can wait for commands
    state = State::IDLE;
    ros::ServiceServer execute_plan_srv = n.advertiseService("execute_plan", set_execute_plan_callback);

    while(ros::ok()) {

        switch (state)
        {
        case State::IDLE:
            // do nothing
            break;

        case State::EXECUTE_PLAN:
            execute_plan();
            break;
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}