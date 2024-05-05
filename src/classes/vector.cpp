#include "vector.h"

float Vector::get_length() const {
    return std::sqrt(pow(x, 2) + pow(y, 2));
}

float Vector::scalar_product(const Vector b) const {
    return x * b.x + y * b.y;
}

float Vector::cross_product(const Vector b) const { 
    return x * b.y - b.x * y; 
} 

Vector::Vector(float x_coordinate, float y_coordinate) {
    x = x_coordinate;
    y = y_coordinate;
}

Vector::Vector() {
    
}