#include "ros/ros.h"
#include <cstdlib>
#include <set>
#include <vector>
#include <list>
#include "../classes/line.h"

static std::vector<Line> perform_ransack(Vector point_array[], int array_size, float epsilon, int min_matches)
{    
    std::vector<Line> discovered_lines;
    std::set<int> covered;
    
    for (int u = 0; u < array_size - 1; u++) {
        if (covered.count(u)) {
            continue;
        }

        for (int v = u + 1; v < array_size; v++) {
            if (covered.count(v)) {
                continue;
            }     

            Vector dif (point_array[v].x - point_array[u].x, point_array[v].y - point_array[u].y);
            Line line(dif, point_array[v]);
            // ROS_DEBUG("Ransack: Line(np.array([%f, %f]), np.array([%f, %f]))", line.m_offset.x, line.m_offset.y, line.m_direction.x, line.m_direction.y);
            // ROS_INFO("Ransack: point(np.array([%f, %f])", point_array[u].x, point_array[u].y);
            // ROS_INFO("Ransack: offset(np.array([%f, %f])", point_array[v].x, point_array[v].y);
            std::list<int> matched;

            for (int w = 0; w < array_size; w++) {
                if (covered.count(w)) {
                    continue;
                }

                float distance = line.get_distance_to_point(point_array[w]);

                if (std::abs(distance) < epsilon) {
                    matched.push_back(w);
                }                
            }

            if (matched.size() < min_matches) {
                continue;
            }

            matched.push_back(u);
            matched.push_back(v);
            discovered_lines.push_back(line);
            covered.insert(matched.begin(), matched.end());
        }
    }

    return discovered_lines;
}