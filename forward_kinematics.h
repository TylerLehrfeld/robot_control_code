#include "Point.h"
#include "Transform.h"
#include "forward_kinematics.cpp"
#ifndef KINEMATICS
#define KINEMATICS


/**
 * @brief Get a left or right joint. The joint is where the proximal links meet
 * the distal links
 *
 * @param is_left_joint
 * @param midpoint_distance
 * @param transmission_length
 * @param proximal_length
 * @param base
 * @param slider
 * @return Point
 */
Point get_joint(bool is_left_joint, float midpoint_distance,
                float transmission_length, float proximal_length, Point base,
                Point slider);

/**
 * @brief Get the end effector of each linkage.
 * The top and bottom have slightly different extentions, but the main math is
 * just getting the tip of an isosceles triangle.
 *
 * @param is_upper
 * @param left_slider
 * @param right_slider
 * @param base
 * @param transmission_length
 * @param proximal_length
 * @param distal_length
 * @param midpoint_distance
 * @param end_effector_vect
 * @param z
 * @return Point
 */
Point get_linkage_end_effector(bool is_upper, Point left_slider,
                               Point right_slider, Point base,
                               float transmission_length, float proximal_length,
                               float distal_length, float midpoint_distance,
                               Point end_effector_vect, float z);

/**
 * @brief Get the end effector (needle tip) based on the top and bottom linkage
 * positions. This
 *
 * @param left
 * @param left_middle
 * @param right_middle
 * @param right
 * @param top_base
 * @param bottom_base
 * @param needle_extension
 * @return Point
 */
Point get_end_effector(Point left, Point left_middle, Point right_middle,
                       Point right, Point top_base, Point bottom_base,
                       float needle_extension);

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