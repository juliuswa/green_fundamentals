#include "ros/ros.h"
#include <cstdlib>
#include <set>
#include <vector>
#include <list>
#include "../classes/line.h"

const int candidate_count = 100;
const int second_index_offset = 10;
const float epsilon = 0.05;
const int min_score = 8;

static Line get_line_candidate(Eigen::Vector2f point_array[]) {
    int point_array_size = sizeof(point_array) / sizeof(point_array[0]);

    int randomIndex1 = rand() % point_array_size;
    int randomIndex2 = randomIndex1 + (rand() % second_index_offset);

    Eigen::Vector2f direction = point_array[randomIndex2] - point_array[randomIndex1];
    return Line(direction, point_array[randomIndex1]);
}

static int evaluate_line(Line line, Eigen::Vector2f point_array[]) {
    int point_array_size = sizeof(point_array) / sizeof(point_array[0]);
    int hits = 0;

    for (int i = 0; i < point_array_size; i++) {
        if (line.get_distance_to_point(point_array[i]) < epsilon) {
            hits += 1;
        }
    }

    return hits;
}

static std::vector<Line> perform_ransac(Eigen::Vector2f point_array[])
{    
    std::vector<Line> lines;

    for (int i = 0; i < candidate_count; i++) {
        Line candidate = get_line_candidate(point_array);
        int score = evaluate_line(candidate, point_array);

        if (score > min_score) {
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