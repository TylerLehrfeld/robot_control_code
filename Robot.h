#ifndef ROBOT
#define ROBOT

#include "Point.h"

class linkage_array {
    public:
    Point base;
    Point left_joint;
    Point right_joint;
    Point left_midpoint;
    Point right_midpoint;
    Point linkage_end_effector;
    Point extended_end_effector;
};

class Robot {
    public:
    Point origin;
    Point x_prime;
    Point y_prime;
    Point z_prime;
    linkage_array top_linkage;
    linkage_array bottom_linkage;
};

#endif