

#include "../NewTransform.h"
#include "../Point.h"
#include "../kinematic_structs.h"

struct Thetas {
    float theta_1;
    float theta_2;
    float theta_3;
    float theta_4;
};

struct Loop_parameters {
    float x_slider_offset;
    float transmission_link_length;
    float proximal_link_length;
    float distal_link_length;
    float needle_to_holder_distance; // only define once
    // float needle_z_offset; //ignore for now
};

struct Parameters {
    Loop_parameters loop_parameters[4];
};

static NewTransform F_O1(0, 0, 0, 0, 224, 0);
static NewTransform F_O2(0, 0, 0, 0, 186, 31.96);

NewTransform get_end_effector(Thetas thetas, Parameters parameters);
