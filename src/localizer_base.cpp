#include "Eigen/Dense"

struct Particle {
    Eigen::Vector2f position;
    float theta;
    float weight = 0.;

    Particle() = default;

    Particle(const Eigen::Vector2f& pos, float t, float w) 
        : position(pos), theta(t), weight(w) {}

    Particle(const Particle& other)
        : position(other.position), theta(other.theta), weight(other.weight) {}

    Particle& operator=(const Particle& other) {
        if (this != &other) {
            position = other.position;
            theta = other.theta;
            weight = other.weight;
        }

        return *this;
    }
};
