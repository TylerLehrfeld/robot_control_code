#include "kinematics.h"
#include <cmath>

NewTransform get_end_effector(Thetas thetas, Parameters parameters) {
    Tunable_parameters tunable_params = parameters.tunable_params;
    Point lower_P = get_lower_linkage_P(thetas, parameters);
    Point upper_P = get_upper_linkage_P(thetas, parameters);
    Point n_vec = (upper_P - parameters.upper_base).normalize();
    Point lower_E =
        (n_vec * tunable_params.bottom_needle_to_holder_distance) + lower_P;
    Point upper_E =
        (n_vec * tunable_params.top_needle_to_holder_distance) + upper_P;
    Point needle_x = (lower_E - upper_E).normalize();
    Point end_effector_point =
        needle_x * (thetas.theta_5 + tunable_params.needle_offset) + lower_E;
    vector<Matrix> columns;
    columns.push_back(needle_x.to_matrix());
    columns.push_back(cross(n_vec, needle_x).to_matrix());
    columns.push_back(n_vec.to_matrix());
    Matrix R(columns);
    Transform EE(R, end_effector_point.to_matrix());
    return EE;
}

Point get_lower_linkage_P(Thetas thetas, Parameters parameters) {
    Point lower_C = get_linkage_C(thetas.theta_2, thetas.theta_3,
                                  parameters.tunable_params.loop_parameters[2],
                                  parameters.tunable_params.loop_parameters[3],
                                  parameters.upper_base);
    Point lower_left_B = get_linkage_B(
        thetas.theta_2, parameters.tunable_params.loop_parameters[2],
        parameters.lower_base);
    Point P =
        (lower_C - lower_left_B).normalize() *
            (parameters.tunable_params.loop_parameters[2].distal_link_length +
             parameters.tunable_params.bottom_needle_to_holder_distance) +
        lower_left_B;
    return P;
}

Point get_upper_linkage_P(Thetas thetas, Parameters parameters) {
    Point upper_C = get_linkage_C(thetas.theta_1, thetas.theta_1,
                                  parameters.tunable_params.loop_parameters[0],
                                  parameters.tunable_params.loop_parameters[1],
                                  parameters.upper_base);
    Point P = (upper_C - parameters.upper_base).normalize() *
                  parameters.tunable_params.top_needle_to_holder_distance +
              upper_C;
    return P;
}

Point get_linkage_C(float left_theta, float right_theta,
                    Loop_parameters left_parameters,
                    Loop_parameters right_parameters, Point base) {
    Point left_B = get_linkage_B(left_theta, left_parameters, base);

    Point right_B = get_linkage_B(right_theta, right_parameters, base);
    Point diff = right_B - left_B;
    float d1 = left_parameters.distal_link_length;
    float d2 = right_parameters.distal_link_length;
    float d3 = diff.magnitude();
    float cos_phi = (d1 * d1 + d3 * d3 - d2 * d2) / (2 * d1 * d3);
    float sin_phi = std::sqrt(1 - cos_phi * cos_phi);
    Point rotated = {.x = diff.x * cos_phi + diff.y * -sin_phi,
                     .y = diff.x * sin_phi + diff.y * cos_phi,
                     .z = 0};
    rotated = rotated.normalize() * d1;
    Point C = rotated + left_B;
    return C;
}

Point get_linkage_B(float theta, Loop_parameters parameters, Point base) {
    Point S = {parameters.x_slider_offset, theta, base.z};
    Point diff = base - S;
    float d1 = parameters.transmission_link_length;
    float d2 = parameters.proximal_link_midpoint;
    float d3 = diff.magnitude();
    float cos_phi = (d1 * d1 + d3 * d3 - d2 * d2) / (2 * d1 * d3);
    float sin_phi = std::sqrt(1 - cos_phi * cos_phi);
    Point rotated = {.x = diff.x * cos_phi + diff.y * -sin_phi,
                     .y = diff.x * sin_phi + diff.y * cos_phi,
                     .z = 0};

    rotated = rotated.normalize() * d1;
    Point M = rotated + S;
    Point B = ((M - base) * (parameters.proximal_link_length /
                             parameters.proximal_link_midpoint)) +
              base;
    return B;
}
