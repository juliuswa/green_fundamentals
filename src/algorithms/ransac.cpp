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
        float distance = line.get_distance_from_point(point_array[i]);
        
        if (distance < epsilon) {
            hits += 1;
        }
    }

    return hits;
}

static std::vector<Line> perform_ransac(std::list<Eigen::Vector2f> points)
{    
    auto compare_by_score = [](const Line& l1, const Line& l2) {
        return l1.m_score > l2.m_score;
    };

    std::vector<Line> lines;
    bool line_found = true;

    while(line_found) {
        Eigen::Vector2f point_array[points.size()];
        std::copy(points.begin(), points.end(), point_array);
        
        std::vector<Line> candidates;

        for (int i = 0; i < candidate_count; i++) {
            Line candidate = get_line_candidate(point_array, points.size());
            candidate.m_score = evaluate_line(candidate, point_array, points.size());
            candidates.push_back(candidate);
        }        

        std::sort(candidates.begin(), candidates.end(), compare_by_score);

        if (candidates[0].m_score < min_score) {
            line_found = false;
            continue;
        }

        lines.push_back(candidates[0]);

        std::list<Eigen::Vector2f> uncovered_points;
        for (int i = 0; i < points.size(); i++) {
            if (candidates[0].get_distance_from_point(point_array[i]) > epsilon) {
                uncovered_points.push_back(point_array[i]);
            }
        } 

        points = uncovered_points;
    }

    return lines;
}