#ifndef ROBOT
#define ROBOT

#include "kinematic_structs.h"


class Robot {
    public:
    Point origin;
    Point x_prime;
    Point y_prime;
    Point z_prime;
    linkage_array top_linkage;
    linkage_array bottom_linkage;
    slider_positions sliders;
};

#endif