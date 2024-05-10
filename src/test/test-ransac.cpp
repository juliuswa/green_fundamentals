#include <gtest/gtest.h>
#include <iostream>
#include <fstream>
#include <cstdlib>
#include "../algorithms/ransack.cpp"

/**
 * This is a test where the robot was placed facing a single straight wall. The wall is perpendicular to the wall
 * with a distance of 30cm. After applying the ransac(k) algorithm a single wall is expected to be detected with the 
 * offset of (30,0) and the direction of (0,1). 
*/

std::list<Vector> get_measurements(std::vestor<Vector>& ranges) 
{
    std::list<Vector> all_vectors;

    for(int i = 0; i < ranges.size(); i++) {
        float r = ranges[i];
        
        if (r != r) {
            continue;
        }  

        float angle_increment = 0.006135923322290182;
        const float theta_offset = 0.74; 

        float theta = i * angle_increment + theta_offset;

        Vector vector(r * std::cos(theta), r * std::sin(theta));

        all_vectors.push_back(vector);
    }
    return all_vectors;
}

std::vector<Vector> getRangesFromFile(const std::string& input_file) {
    std::ifstream file(input_file);
    std::vestor<Vector> ranges;
    float value;

    if(!file.is_open()) std::cerr << "Error: Could not open input for test" << std::endl;

    while(file >> value) {
        ranges.push_back(value);
    }
    file.close;
}

TEST(RansacTest, Perpendicular30) {
    Vector expected_direction(0,1);
    Vector expected_offset(30,0);
    const float epsilon = 0.1;  // private values from line
    const int min_matches = 2;

    std::vector<Vector> ranges = getRangesFromFile("../test-input/point_input_30.txt");
    // TODO points, epsilon, min_matches
    std::list<Vector> measurements = get_measurements(ranges);
    Vector point_array[measurements.size()];
    std::copy(measurements.begin(), measurements.end(), point_array);

    std::vector<Line> discovered_lines = perform_ransack(point_array, point_array.size(), epsilon, min_matches);

    ASSERT_FALSE(discovered_lines.empty());

    std::cout << "Discovered Lines:" << std::endl;
    for (const auto& line : discovered_lines) {
        Vector direction = line.getDirection();
        Vector offset = line.getOffset();
        std::cout << "Line Direction: (" << direction.x << ", " << direction.y << ")" << std::endl;
        std::cout << "Line Offset: (" << offset.x << ", " << offset.y << ")" << std::endl;
        
        ASSERT_NEAR(direction.x, expected_direction.x, epsilon);
        ASSERT_NEAR(direction.y, expected_direction.y, epsilon);
        ASSERT_NEAR(offset.x, expected_offset.x, epsilon);
        ASSERT_NEAR(offset.x, expected_offset.y, epsilon);
    }
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
