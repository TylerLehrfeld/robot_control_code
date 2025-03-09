#include "Point.h"
#include "forward_kinematics.cpp"
#include <cassert>


bool is_close(double a, double b) {
    return a - .0001 < b && a + .0001 > b;
}

/**
 * @brief This test the get joint method which works on links. We assume that the forward direction of the robot is y, up is z, and right is x.
 * Tests were created using pencil and paper
 */
void test_get_joint() {
    Point p = get_joint(true, LOWER_MIDPOINT_DISTANCE, LOWER_TRANSMISSION_LENGTH,
        LOWER_PROXIMAL_LENGTH, {0, 10, 0}, {-5, 5, 0});
    assert(is_close(p.x,  -8.08498));
    assert(is_close(p.y, 15.88498));
    assert(is_close(p.z, 0.0));
    Point p2 = get_joint(false, 6, 9, 12, {0, 10, 0}, {2, 4, 0});
    assert(is_close(p2.x,  11.10947));
    assert(is_close(p2.y, 14.53648));
    assert( is_close(p2.z, 0.0));
}


void test_get_linkage_end_effector() {
    Point p = get_linkage_end_effector(true, {-5,5,0}, {5,6,0},{0,10,0}, UPPER_TRANSMISSION_LENGTH, UPPER_PROXIMAL_LENGTH, UPPER_DISTAL_LENGTH, UPPER_MIDPOINT_DISTANCE, {0,.5,0}, 0);
}

int main() {
    //test_get_joint();
    //test_get_linkage_end_effector();
    Point end_effector = get_end_effector({-63, 109.08, 0}, {-21,144.91 ,0}, {21.07, 150.71, 0}, {63, 109.08, 0}, {0, 186.4, 0}, {0, 223.62, 0}, 0);
    std::cout << "end effector" << std::endl;
    end_effector.print();
}
