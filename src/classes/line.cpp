//
// Created by laura on 03.05.24.
//

#include "line.h"


Line::Line(float unnormed_x, float unnormed_y, int offset_point) {
    float norm = std::sqrt(pow(unnormed_x, 2) + pow(unnormed_y, 2));

    x = unnormed_x / norm;
    y = unnormed_y / norm;

    offset = offset_point;
}