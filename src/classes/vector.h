#ifndef GREEN_FUNDAMENTALS_VECTOR_H
#define GREEN_FUNDAMENTALS_VECTOR_H

#include <cmath>
#include <cstdlib>

class Vector {
public:
    float x;
    float y;

    Vector(float x, float y);
    Vector();

    float get_length();
    float scalar_product(Vector b);
};


#endif //GREEN_FUNDAMENTALS_VECTOR_H
