#include "inverse_kinematics.h"
#include "forward_kinematics.h"
#include "kinematic_structs.h"
#include <cassert>
#include <cmath>
#include <iostream>
#include <ostream>


void test_overall_kinematics() {

    /*double left_slider_y = BASE_TO_SLIDER_MAX-66.435;
    double left_middle_slider_y =  BASE_TO_SLIDER_MAX-32.463;
    double right_middle_slider_y =BASE_TO_SLIDER_MAX-27.424;
    double right_slider_y = BASE_TO_SLIDER_MAX-66.435;*/
    
    /*double left_slider_y = BASE_TO_SLIDER_MAX-41;
    double left_middle_slider_y =  BASE_TO_SLIDER_MAX-11;
    double right_middle_slider_y =BASE_TO_SLIDER_MAX-11;
    double right_slider_y = BASE_TO_SLIDER_MAX-41;
    
    Point left = {sliderXs[0], left_slider_y, 0};
    Point left_middle = {sliderXs[1],left_middle_slider_y, 0};
    Point right_middle = {sliderXs[2], right_middle_slider_y, 0};
    Point right = {sliderXs[3], right_slider_y, 0};
    std::cout << "forward kinematics" << std::endl;
    Point needle_end_effector = get_end_effector(left, left_middle, right_middle,
        right, UPPER_BASE, LOWER_BASE,
        0);
    std::cout << "end effector" << std::endl;
    needle_end_effector.print();
    //we are getting a point at the correct angle, and above the target point so we can feed it into inverse kinematics
    Point injection_point = get_end_effector(left, left_middle, right_middle,
            right, UPPER_BASE, LOWER_BASE,
            -1);
    std::cout <<"inverse kinematics" << std::endl;
    slider_positions positions = inverse_kinematics({needle_end_effector, injection_point});
    assert(isclose(positions.left_slider_y, left_slider_y));
    assert(isclose(positions.left_middle_slider_y, left_middle_slider_y));
    assert(isclose(positions.right_middle_slider_y, right_middle_slider_y));
    assert(isclose(positions.right_slider_y, right_slider_y));
    std::cout << "assertions passed" << std::endl;*/

    /*double left_slider_y = BASE_TO_SLIDER_MAX-66.435;
    double left_middle_slider_y =  BASE_TO_SLIDER_MAX-32.463;
    double right_middle_slider_y =BASE_TO_SLIDER_MAX-27.424;
    double right_slider_y = BASE_TO_SLIDER_MAX-66.435;
    Point left = {sliderXs[0], left_slider_y, 0};
    Point left_middle = {sliderXs[1],left_middle_slider_y, 0};
    Point right_middle = {sliderXs[2], right_middle_slider_y, 0};
    Point right = {sliderXs[3], right_slider_y, 0};

    get_end_effector(left, left_middle, right_middle, right, UPPER_BASE, LOWER_BASE, 0).print();*/
    
    approach_definition def = {{0,450,-65},M_PI/2,M_PI/6};
    Transform t(0,0,0,0,0,0);
    slider_positions injection_positions = inverse_kinematics(def, t);
    Point left = {sliderXs[0], injection_positions.left_slider_y, 0};
    Point left_middle = {sliderXs[1],injection_positions.left_middle_slider_y, 0};
    Point right_middle = {sliderXs[2], injection_positions.right_middle_slider_y, 0};
    Point right = {sliderXs[3], injection_positions.right_slider_y, 0};
    
    get_end_effector(left, left_middle, right_middle, right, UPPER_BASE, LOWER_BASE, injection_positions.needle_extension).print();
    injection_positions.print();
}

int main() {
    test_overall_kinematics();
}