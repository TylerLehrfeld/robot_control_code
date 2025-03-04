#include "3D_slicer_interface.h"
#include "Point.h"
#include "Transform.h"
#ifndef KINEMATICS
#define KINEMATICS

const float UPPER_TRANSMISSION_LENGTH = 115;
const float UPPER_PROXIMAL_LENGTH = 116;
const float UPPER_DISTAL_LENGTH = 125;
const float UPPER_MIDPOINT_DISTANCE = 58;
const float LOWER_TRANSMISSION_LENGTH = 110;
const float LOWER_PROXIMAL_LENGTH = 92;
const float LOWER_DISTAL_LENGTH = 99;
const float LOWER_MIDPOINT_DISTANCE = 46;
const float UPPER_LINKAGE_Z = 31.96;
const float LOWER_LINKAGE_Z = 0;
const Point LOWER_END_EFFECTOR_TO_NEEDLEPOINT = {.x = 0, .y = 47.7, .z = -64.9};
const float UPPER_TO_LOWER_BASE_LENGTH = 38;
const Point LOWER_BASE = {0, 186 + UPPER_TO_LOWER_BASE_LENGTH, 0};
const Point UPPER_BASE = {0, 186, 0};
const Point UPPER_END_EFFECTOR_VECTOR = {0, 19.11, 0};
const Point LOWER_END_EFFECTOR_VECTOR = {14, 0, 0};
const float sliderXs[4] = {-63, -21, 21, 63};


/**
 * @brief a struct that contains the y positions of all the sliders
 * 
 */
 struct slider_positions {
  float left_slider_y;
  float left_middle_slider_y;
  float right_middle_slider_y;
  float right_slider_y;
};

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

  void get_slider_positions(slider_positions& slider_positions_struct, Point end_effector, bool upper);

#endif