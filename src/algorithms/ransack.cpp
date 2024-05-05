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

            Line line(point_array[u], point_array[v]);
            std::list<int> matched;

            for (int w = 0; w < array_size; w++) {
                if (covered.count(w)) {
                    continue;
                }

                float distance = line.get_distance_to_point(point_array[w]);

                if (distance < epsilon) {
                    matched.push_back(w);
                }
                
            }

            if (matched.size() < min_matches) {
                continue;
            }

            discovered_lines.push_back(line);
            covered.insert(matched.begin(), matched.end());
        }
    }

    return discovered_lines;
}