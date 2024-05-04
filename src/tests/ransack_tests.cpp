#include <iostream>
#include "../algorithms/ransack.cpp"
#include "../classes/vector.h"

bool test_ransack() {

    Vector p1(0.2, 0.3);
    Vector p2(0.4, 0.3);
    Vector p3(0.0, 0.2);
    Vector p4(-0.2, 0.1);
    Vector p5(0.4, 0.1);
    Vector p6(-0.4, 0.0);
    Vector p7(0.4, -0.1);
    Vector p8(0.4, -0.3);
    
    Vector points[] = {p1, p2, p3, p4, p5, p6, p7, p8};
    std::list<Line> lines = perform_ransack(points, 0.1, 3);

    int i = 0;
    return true;
}

int main() {
    bool success = test_ransack();
    std::cout << "testing ransack " << (success ? "successful" : "failed") << std::endl;
}
