#include "Point.h"
#include "Transform.h"
#include "kinematic_structs.h"

#ifndef INVERSE_KINEMATICS
#define INVERSE_KINEMATICS

Point intersection_of_two_circles(Point center_a, Point center_b,
                                  float radius_a, float radius_b,
                                  bool left_point);

Point get_linkage_end_effector(
    target_and_injection_point_approach robot_approach, float z,
    bool is_upper_linkage);

void get_slider_positions(slider_positions &slider_positions_struct,
                          Point end_effector, bool upper);

slider_positions inverse_kinematics(
    target_and_injection_point_approach needle_to_patient_approach);

slider_positions inverse_kinematics(
    target_and_injection_point_approach needle_to_patient_approach,
    Transform T_RP);

#endif