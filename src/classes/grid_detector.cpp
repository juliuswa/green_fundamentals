// Given:
// data – A set of observations.
// model – A model to explain the observed data points.
// n – The minimum number of data points required to estimate the model parameters.
// k – The maximum number of iterations allowed in the algorithm.
// t – A threshold value to determine data points that are fit well by the model (inlier).
// d – The number of close data points (inliers) required to assert that the model fits well to the data.

// Return:
// bestFit – The model parameters which may best fit the data (or null if no good model is found).


// iterations = 0
// bestFit = null
// bestErr = something really large // This parameter is used to sharpen the model parameters to the best data fitting as iterations go on.

// while iterations < k do
// maybeInliers := n randomly selected values from data
// maybeModel := model parameters fitted to maybeInliers
//         confirmedInliers := empty set
// for every point in data do
// if point fits maybeModel with an error smaller than t then
// add point to confirmedInliers
// end if
// end for
// if the number of elements in confirmedInliers is > d then
// // This implies that we may have found a good model.
// // Now test how good it is.
// betterModel := model parameters fitted to all the points in confirmedInliers
//         thisErr := a measure of how well betterModel fits these points
// if thisErr < bestErr then
//         bestFit := betterModel
//         bestErr := thisErr
//         end if
// end if
// increment iterations
// end while

// return bestFit

#include "grid_detector.h"


struct Point {
    float x;
    float y;
};

int offset_of_laser = 13;

std::vector<Point> get_cartesian_points(const sensor_msgs::LaserScan::ConstPtr& laser_scan){
    std::vector<Point> all_points(laser_scan->ranges.size());
    for(int i = 0; i < laser_scan->ranges.size(); i++) {
        float r = laser_scan->ranges[i];
        float theta = i * laser_scan->angle_increment;

        if (r != r) {
            continue;
        }

        Point point = {r * std::cos(theta), r * std::sin(theta)};
        all_points.push_back(point);
    }
    return all_points;
}



void GridDetector::detect_grid(const sensor_msgs::LaserScan::ConstPtr& laser_scan) {

    ROS_INFO("detect_grid is called.");
    std::vector<Point> points = get_cartesian_points(laser_scan);

    ROS_DEBUG("%d points detected", points.size());

    // ROS_DEBUG("%s", generateSpace(points).c_str());
}

GridDetector::GridDetector() {}


// TODO: Delete
// std::string generateSpace(const std::vector<Point>& points) {
//     // Find min and max x, y coordinates
//     float minX = std::numeric_limits<float>::max();
//     float maxX = std::numeric_limits<float>::min();
//     float minY = std::numeric_limits<float>::max();
//     float maxY = std::numeric_limits<float>::min();

//     for (const auto& p : points) {
//         minX = std::min(minX, p.x);
//         maxX = std::max(maxX, p.x);
//         minY = std::min(minY, p.y);
//         maxY = std::max(maxY, p.y);
//     }

//     // Calculate step sizes for x and y
//     float stepX = (maxX - minX) / 100.0;
//     float stepY = (maxY - minY) / 100.0;

//     // Initialize the grid with spaces
//     std::string grid(100 * 100, ' ');

//     // Mark points with 'X'
//     for (const auto& p : points) {
//         int xIndex = static_cast<int>((p.x - minX) / stepX);
//         int yIndex = static_cast<int>((p.y - minY) / stepY);

//         grid[yIndex * 100 + xIndex] = 'X';
//     }

//     // Convert grid to a string with line breaks
//     std::string result;
//     for (int y = 0; y < 100; ++y) {
//         result += grid.substr(y * 100, 100) + '\n';
//     }

//     return result;
// }

