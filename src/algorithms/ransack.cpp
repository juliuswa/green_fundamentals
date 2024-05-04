#include <cstdlib>
#include <list>
#include <set>
#include "../classes/line.h"

static std::list<Line> perform_ransack(std::list<Vector> points, float epsilon, int min_matches)
{    
    Vector point_array[points.size()];
    std::copy(points.begin(), points.end(), point_array);

    std::list<Line> discovered_lines;
    std::set<int> covered;
    
    for (int u = 0; u < points.size() - 1; u++) {
        if (covered.count(u)) {
            continue;
        }

        for (int v = u + 1; v < points.size(); v++) {
            if (covered.count(v)) {
                continue;
            }     

            Line line(point_array[u], point_array[v]);
            std::list<int> matched;

            for (int w = 0; w < points.size(); w++) {
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