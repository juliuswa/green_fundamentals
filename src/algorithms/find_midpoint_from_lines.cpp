#include "../Eigen/Dense"
#include "../classes/line.h"
#include "../classes/vector.h"
#include <math.h>
#include <cfloat>

// Vergleich https://colab.research.google.com/drive/1EEDDlQFIfz_srAhyCRq71XHXUHVcb7Vq?usp=sharing

static float angle_between_vectors(const Vector& a, const Vector& b) {
    float dot = a.scalar_product(b);
    float len_a = a.get_length();
    float len_b = b.get_length();
    return acos(dot / (len_a * len_b)); // clip angle between 1.0 and -1.0
}

static Vector find_line_intersection(const Line& line1, const Line& line2) {
    Eigen::Matrix2f A;
    Eigen::Vector2f b;

    // Fill the matrix A and vector b based on the lines' orientation and position
    A << line1.m_direction.x, -line2.m_direction.x,
         line1.m_direction.y, -line2.m_direction.y; // Should be a matrix
    b << line2.m_offset.x - line1.m_offset.x,
         line2.m_offset.y - line1.m_offset.y; // Should be a vector

    // Solve the linear system A * [t, s] = b
    if (!A.fullPivLu().isInvertible()) {
        // If the matrix is singular (lines are parallel or coincident), return a zero vector to indicate no valid intersection
        return Vector(FLT_MAX, FLT_MAX);
    }
    Eigen::Vector2f ts = A.fullPivLu().solve(b);

    float t = ts[0];

    // Calculate the intersection point
    Vector intersection(line1.m_offset.x + t * line1.m_direction.x, line1.m_offset.y + t * line1.m_direction.y);

    return intersection;
}

static Eigen::Vector2f toEigenVector(const Vector& v) {
    return Eigen::Vector2f(v.x, v.y);
}

static std::pair<Line, Vector> displace_line(const Line& line, const Vector& point, float distance = 0.39f) {
    Eigen::Vector2f A = toEigenVector(line.m_offset);
    Eigen::Vector2f B = toEigenVector(line.m_offset) + toEigenVector(line.m_direction);
    Eigen::Vector2f P = toEigenVector(point);

    Eigen::Vector2f perpendicular_vector1(-B[1] + A[1], B[0] - A[0]);
    perpendicular_vector1.normalize(); // Normalizes the vector

    Eigen::Vector2f perpendicular_vector2(B[1] - A[1], -B[0] + A[0]);
    perpendicular_vector2.normalize(); // Normalizes the vector

    if ((A + perpendicular_vector1 - P).norm() < (A + perpendicular_vector2 - P).norm()) {
        // perpendicular_vector1 is closer to the point
        Vector displacement1(perpendicular_vector1[0] * distance, perpendicular_vector1[1] * distance);
        Vector new_offset = Vector(line.m_offset.x + displacement1.x, line.m_offset.y + displacement1.y);
        return {Line(new_offset, line.m_offset), displacement1};
    } else {
        // perpendicular_vector2 is closer to the point
        Vector displacement2(perpendicular_vector2[0] * distance, perpendicular_vector2[1] * distance);
        Vector new_offset = Vector(line.m_offset.x + displacement2.x, line.m_offset.y + displacement2.y);
        return {Line(new_offset, line.m_offset), displacement2};
    }
}


static float rad_to_deg(float rad) 
{
    return rad * 180./M_PI;
}

static std::pair<Vector, Vector> get_midpoint_from_lines(
    const std::vector<Line>& lines, 
    const Vector& sensor, 
    float threshold_degrees = 15.0) 
{
    
    // 1.  Shift lines in sensor direction
    std::vector<Line> shifted_lines;
    Vector perp; // Assuming perp is calculated somewhere within displace_line

    for (const Line& line : lines) {
        std::pair<Line, Vector> temp_pair = displace_line(line, sensor);
        Line shifted = temp_pair.first;
        perp = temp_pair.second;
        shifted_lines.push_back(shifted);
    }

    // 2. Find Line pairs
    std::vector<std::pair<Line, Line>> line_pairs;
    for (size_t i = 0; i < shifted_lines.size(); ++i) {
        for (size_t j = i + 1; j < shifted_lines.size(); ++j) {
            float angle_degrees = std::abs(rad_to_deg(angle_between_vectors(shifted_lines[i].m_direction, shifted_lines[j].m_direction)) - 90);
            if (angle_degrees < threshold_degrees) {
                line_pairs.emplace_back(shifted_lines[i], shifted_lines[j]);
            }
        }
    }

    // 3. Find Intersections
    std::vector<Vector> intersections;
    for (const auto& pair : line_pairs) {
        Vector intersection = find_line_intersection(pair.first, pair.second);
        if (intersection.x != FLT_MAX && intersection.y != FLT_MAX) { 
            intersections.emplace_back(intersection);
        }
    }

    // 4. Find Mean of Intersections
    Vector average_point = Vector(0., 0.);
    if (!intersections.empty()) {
        for (const Vector& inter : intersections) {
            average_point.x += inter.x;
            average_point.y += inter.y;
        }
        average_point.x /= intersections.size();
        average_point.y /= intersections.size();

    Vector new_perp = Vector(perp.x * -1, perp.y * -1);
    
    return {average_point, new_perp};
}