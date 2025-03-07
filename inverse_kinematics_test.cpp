#include "inverse_kinematics.h"
#include "forward_kinematics.h"
#include <cassert>

void test_overall_kinematics() {
    float left_slider_y = 106.565;
    float left_middle_slider_y =  BASE_TO_SLIDER_MAX-34.5634;
    float right_middle_slider_y =BASE_TO_SLIDER_MAX-29.8236;
    float right_slider_y = 106.565;
    
    Point left = {sliderXs[0], left_slider_y, 0};
    Point left_middle = {sliderXs[1],left_middle_slider_y, 0};
    Point right_middle = {sliderXs[2], right_middle_slider_y, 0};
    Point right = {sliderXs[3], right_slider_y, 0};
    Point needle_end_effector = get_end_effector(left, left_middle, right_middle,
        right, UPPER_BASE, LOWER_BASE,
        0);
    //we are getting a point at the correct angle, and above the target point so we can feed it into inverse kinematics
    Point injection_point = get_end_effector(left, left_middle, right_middle,
            right, UPPER_BASE, LOWER_BASE,
            -1);
    slider_positions positions = inverse_kinematics({needle_end_effector, injection_point});
    assert(positions.left_slider_y == left_slider_y);
    assert(positions.left_middle_slider_y == left_middle_slider_y);
    assert(positions.right_middle_slider_y == right_middle_slider_y);
    assert(positions.right_slider_y == right_slider_y);
}

int main() {
    test_overall_kinematics();
}