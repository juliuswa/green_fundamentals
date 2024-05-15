#include "ros/ros.h"
#include <cstdlib>
#include <set>
#include <vector>
#include <list>
#include "../classes/line.h"

const int candidate_count = 100;
const int second_index_offset = 10;
const float epsilon = 0.02;
const int min_score = 50;

static Line get_line_candidate(Eigen::Vector2f point_array[], int point_array_size) {
    int randomIndex1 = rand() % point_array_size;
    int randomIndex2 = (randomIndex1 + (rand() % second_index_offset)) % point_array_size;

    Eigen::Vector2f direction = point_array[randomIndex2] - point_array[randomIndex1];
    return Line(direction, point_array[randomIndex1]);
}

static int evaluate_line(Line line, Eigen::Vector2f point_array[], int point_array_size) {
    int hits = 0;

    for (int i = 0; i < point_array_size; i++) {
        float distance = std::abs(line.get_distance_from_point(point_array[i]));
        
        if (distance < epsilon) {
            hits += 1;
        }
    }

    return hits;
}

static std::vector<Line> perform_ransac(Eigen::Vector2f point_array[], int point_array_size)
{    
    std::vector<Line> lines;

    for (int i = 0; i < candidate_count; i++) {
        Line candidate = get_line_candidate(point_array, point_array_size);
        candidate.m_score = evaluate_line(candidate, point_array, point_array_size);

        if (candidate.m_score > min_score) {
            lines.push_back(candidate);
        }
    }

    return lines;
    
    // for (int u = 0; u < array_size - 1; u++) {
    //     if (covered.count(u)) {
    //         continue;
    //     }

    //     for (int v = u + 1; v < array_size; v++) {
    //         if (covered.count(v)) {
    //             continue;
    //         }     

    //         Vector dif (point_array[v].x - point_array[u].x, point_array[v].y - point_array[u].y);
    //         Line line(dif, point_array[v]);
    //         // ROS_DEBUG("Ransack: Line(np.array([%f, %f]), np.array([%f, %f]))", line.m_offset.x, line.m_offset.y, line.m_direction.x, line.m_direction.y);
    //         // ROS_INFO("Ransack: point(np.array([%f, %f])", point_array[u].x, point_array[u].y);
    //         // ROS_INFO("Ransack: offset(np.array([%f, %f])", point_array[v].x, point_array[v].y);
    //         std::list<int> matched;

    //         for (int w = 0; w < array_size; w++) {
    //             if (covered.count(w)) {
    //                 continue;
    //             }

    //             float distance = line.get_distance_to_point(point_array[w]);

    //             if (std::abs(distance) < epsilon) {
    //                 matched.push_back(w);
    //             }                
    //         }

    //         if (matched.size() < min_matches) {
    //             continue;
    //         }

    //         matched.push_back(u);
    //         matched.push_back(v);
    //         discovered_lines.push_back(line);
    //         covered.insert(matched.begin(), matched.end());
    //     }
    // }
}