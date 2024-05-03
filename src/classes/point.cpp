#include "point.h"

Point::Point(float a, float b, bool is_polar) {
    if(is_polar) {
        r = a;
        theta = b;

        x = r * std::cos(theta);
        y = r * std::sin(theta);
    } else {
        x = a;
        y = b;

        r = std::sqrt(std::pow(x, 2) + std::pow(y, 2));
        theta = atan(y / x);
    }
};