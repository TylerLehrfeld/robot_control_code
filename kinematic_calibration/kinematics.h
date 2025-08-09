#ifndef KINEMATICS
#define KINEMATICS

#include "../NewTransform.h"
#include "../Point.h"
#include "../kinematic_structs.h"

struct Thetas {
    float theta_1;
    float theta_2;
    float theta_3;
    float theta_4;
    float theta_5;
};

struct Loop_parameters {
    float x_slider_offset;
    float transmission_link_length;
    float proximal_link_length;
    float proximal_link_midpoint;
    float distal_link_length;
};

struct Tunable_parameters {

    /**
     * 0: top left
     * 1: top right
     * 2: bottom left
     * 3: bottom right
     **/
    Loop_parameters loop_parameters[4];
    float top_needle_to_holder_distance;
    float bottom_needle_to_holder_distance;
    float top_holder_to_linkage_distance;
    float bottom_holder_to_linkage_distance;
    float needle_offset;
};
struct Parameters {
    Tunable_parameters tunable_params;
    Point upper_base;
    Point lower_base;
};

static NewTransform F_O1(0, 0, 0, 0, 224, 0);
static NewTransform F_O2(0, 0, 0, 0, 186, 31.96);

NewTransform get_end_effector(Thetas thetas, Parameters parameters);

Point get_upper_linkage_P(Thetas thetas, Parameters parameters);

Point get_lower_linkage_P(Thetas thetas, Parameters parameters);

Point get_linkage_C(float left_theta, float right_theta, Loop_parameters left_parameters,
                    Loop_parameters right_parameters, Point base);

Point get_linkage_B(float theta, Loop_parameters parameters, Point base);
#endif // KINEMATICS
