#include "3D_slicer_interface.h"
#include "Point.h"
#include "Transform.h"
#include "galil_control_calls.cpp"
#include "kinematics.h"
#include <cassert>
#include <cmath>

/**
 * @brief Get the slider positions of the robot based on the approach angle to a
 * point RELATIVE TO THE PATIENT.
 *
 * @param needle_to_patient_approach A struct that defines a point and approach
 * angle for the needle path
 * @param T_RP Transform from Patient frame to robot frame
 * (T_RP*v_patient=v_robot)
 * @return slider_positions
 */
slider_positions
inverse_kinematics(approach_definition needle_to_patient_approach,
                   Transform T_RP) {
  Point mock_injection_point = {sin(needle_to_patient_approach.phi) *
                                    cos(needle_to_patient_approach.theta),
                                sin(needle_to_patient_approach.phi) *
                                    sin(needle_to_patient_approach.theta),
                                cos(needle_to_patient_approach.theta)};
  mock_injection_point =
      mock_injection_point + needle_to_patient_approach.target;
  return inverse_kinematics(
      {T_RP * needle_to_patient_approach.target, T_RP * mock_injection_point});
}

/**
 * @brief Get the slider positions of the robot based on a target and injection
 * point RELATIVE TO THE PATIENT.
 *
 * @param needle_to_patient_approach A struct that defines target and injection
 * points
 * @param T_RP Transform from Patient frame to robot frame
 * (T_RP*v_patient=v_robot)
 * @return slider_positions
 */
slider_positions inverse_kinematics(
    target_and_injection_point_approach needle_to_patient_approach,
    Transform T_RP) {
  return inverse_kinematics({T_RP * needle_to_patient_approach.injection_point,
                      T_RP * needle_to_patient_approach.target});
}

/**
 * @brief Get the slider positions of the robot based on the injection point and
 * target in ROBOT COORDINATES
 *
 * @param needle_to_patient_approach
 * @return slider_positions
 */
slider_positions inverse_kinematics(
    target_and_injection_point_approach needle_to_patient_approach) {
  Point upper_linkage_end_effector = get_linkage_end_effector(
      needle_to_patient_approach, UPPER_LINKAGE_Z, true);
  Point lower_linkage_end_effector = get_linkage_end_effector(
      needle_to_patient_approach, LOWER_LINKAGE_Z, false);
  slider_positions sliders;
  get_slider_positions(sliders, lower_linkage_end_effector, false);
  get_slider_positions(sliders, upper_linkage_end_effector, true);
  return sliders;
}

/**
 * @brief Get the linkage end effector based on its z value
 *
 * @param robot_approach
 * @param z z value of end effector (0 or LOWER_LINKAGE_Z for lower,
 * UPPER_LINKAGE_Z for upper)
 * @param is_upper_linkage boolean flag for upper linkage vs lower linkage
 * (upper: true, lower: false)
 * @return Point
 */
Point get_linkage_end_effector(
    target_and_injection_point_approach robot_approach, float z,
    bool is_upper_linkage) {
  float t = (z - robot_approach.target.z) /
            (robot_approach.injection_point.z - robot_approach.target.z);
  Point end_effector =
      ((robot_approach.injection_point - robot_approach.target) * t) +
      robot_approach.target;
  assert(end_effector.z == z);
  end_effector.z = 0;
  if (is_upper_linkage) {
    Point upper_base = UPPER_BASE;
    upper_base.z = 0;
    Point upper_end_effector_vector = UPPER_END_EFFECTOR_VECTOR;
    end_effector = ((end_effector - upper_base) *
                    ((end_effector - upper_base).magnitude() -
                     upper_end_effector_vector.magnitude())) +
                   upper_base;
  } else {
    Point lower_end_effector_vector = LOWER_END_EFFECTOR_VECTOR;
    Point left_joint = intersection_of_two_circles(
        end_effector, LOWER_BASE,
        LOWER_DISTAL_LENGTH + lower_end_effector_vector.magnitude(),
        LOWER_PROXIMAL_LENGTH, true);
    end_effector.z = 0;
    end_effector =
        (end_effector - left_joint).normalize() * LOWER_DISTAL_LENGTH +
        left_joint;
    end_effector.z = z;
  }
  return end_effector;
}

// Algorithm taken from https://paulbourke.net/geometry/circlesphere/
/**
 * @brief Get the points at the intersection of two circles. WARNING: only call
 * when you know there are two points of intersection. Only takes into account x
 * and y coordinates.
 *
 * @param center_a center point of the first circle
 * @param center_b center point of the second circle
 * @param radius_a radius of the first circle
 * @param radius_b radius of the second circle
 * @param left_point boolean flag if you want the leftmost (closest x to -inf):
 * true, or right point: false
 * @return Point
 */
Point intersection_of_two_circles(Point center_a, Point center_b,
                                  float radius_a, float radius_b,
                                  bool left_point) {
  center_a.z = 0;
  center_b.z = 0;
  float d = (center_a - center_b).magnitude();
  float a = (pow(radius_a, 2) - pow(radius_b, 2) + pow(d, 2)) / (2 * d);
  float h = std::sqrt(pow(radius_a, 2) - pow(a, 2));
  Point P2 = (center_b - center_a) * (a / d) + center_a;
  Point P3A = {.x = P2.x + h * (center_b.y - center_a.y) / d,
               .y = P2.y + h * (center_b.x - center_a.x) / d,
               .z = 0};
  Point P3B = {.x = P2.x - h * (center_b.y - center_a.y) / d,
               .y = P2.y - h * (center_b.x - center_a.x) / d,
               .z = 0};
  if ((left_point && P3A.x < P3B.x) || (!left_point && P3A.x > P3B.x)) {
    return P3A;
  } else {
    return P3B;
  }
}

/**
 * @brief Get the y of slider based on midpoint location on the proxmial link.
 * This essentially solves for y of a circle of radius transmission and center
 * midpoint as a function of x.
 *
 * @param midpoint
 * @param x static x coordinate of the slider
 * @param transmission_length
 * @return float
 */
float get_y_of_slider_based_on_midpoint_location(Point midpoint, float x,
                                                 float transmission_length) {
  assert(midpoint.z == 0);
  return -1 * sqrt(pow(transmission_length, 2) - pow(x - midpoint.x, 2)) +
         midpoint.y;
}

/**
 * @brief Get the slider positions for a linkage based on the end effector
 * position.
 *
 * @param slider_positions_struct
 * @param end_effector
 * @param upper
 */
void get_slider_positions(slider_positions &slider_positions_struct,
                          Point end_effector, bool upper) {
  end_effector.z = 0;
  Point left_joint = intersection_of_two_circles(
      end_effector, upper ? UPPER_BASE : LOWER_BASE,
      upper ? UPPER_DISTAL_LENGTH : LOWER_DISTAL_LENGTH,
      upper ? UPPER_PROXIMAL_LENGTH : LOWER_PROXIMAL_LENGTH, true);
  Point right_joint = intersection_of_two_circles(
      end_effector, upper ? UPPER_BASE : LOWER_BASE,
      upper ? UPPER_DISTAL_LENGTH : LOWER_DISTAL_LENGTH,
      upper ? UPPER_PROXIMAL_LENGTH : LOWER_PROXIMAL_LENGTH, false);
  Point base = upper ? UPPER_BASE : LOWER_BASE;
  float midpoint_distance =
      upper ? UPPER_MIDPOINT_DISTANCE : LOWER_MIDPOINT_DISTANCE;
  Point left_midpoint =
      ((left_joint - base).normalize() * LOWER_MIDPOINT_DISTANCE) + base;
  Point right_midpoint =
      ((right_joint - base).normalize() * LOWER_MIDPOINT_DISTANCE) + base;
  if (upper) {
    slider_positions_struct.left_slider_y =
        get_y_of_slider_based_on_midpoint_location(left_midpoint, sliderXs[0],
                                                   UPPER_TRANSMISSION_LENGTH);
    slider_positions_struct.right_slider_y =
        get_y_of_slider_based_on_midpoint_location(right_midpoint, sliderXs[3],
                                                   UPPER_TRANSMISSION_LENGTH);
  } else {
    slider_positions_struct.left_middle_slider_y =
        get_y_of_slider_based_on_midpoint_location(left_midpoint, sliderXs[1],
                                                   LOWER_TRANSMISSION_LENGTH);
    slider_positions_struct.right_middle_slider_y =
        get_y_of_slider_based_on_midpoint_location(right_midpoint, sliderXs[2],
                                                   LOWER_TRANSMISSION_LENGTH);
  }
}