#ifndef GREEN_FUNDAMENTALS_VECTOR_H
#define GREEN_FUNDAMENTALS_VECTOR_H

#include <cmath>

class Vector {
public:
    float x;
    float y;

    Vector(float x, float y);
    Vector();

    float get_length() const;
    float scalar_product(const Vector b) const;
    float cross_product(const Vector b) const; 
};

#endif //GREEN_FUNDAMENTALS_VECTOR_H
