#include "kinematics.h"
#include "Point.h"
#include <cassert>
#include <cmath>

Point get_joint(bool is_left_joint, float midpoint_distance,
                float transmission_length, float proximal_length, Point base,
                Point slider) {
  float theta =
      std::acos((pow(midpoint_distance, 2) - pow(transmission_length, 2) -
                 pow((base - slider).magnitude(), 2)) /
                (-2 * transmission_length * (base - slider).magnitude()));
  if (!is_left_joint) {
    theta *= -1;
  }
  Transform rotate_z(0, 0, theta, 0, 0, 0);
  Point slider_to_base_vec = (base - slider).normalize();
  Point rotated_left_to_base_vec = rotate_z * slider_to_base_vec;
  Point midpoint = transmission_length * rotated_left_to_base_vec + slider;
  if(is_left_joint) {
    //std::cout << "left midpoint" << std::endl;
  } else {
    //std::cout << "right midpoint" << std::endl;
  }
  //midpoint.print();
   
  Point base_to_midpoint_vec = (midpoint - base).normalize();
  Point joint = proximal_length * base_to_midpoint_vec + base;
  return joint;
}

Point get_linkage_end_effector(bool is_upper, Point left_slider,
                               Point right_slider, Point base,
                               float transmission_length, float proximal_length,
                               float distal_length, float midpoint_distance,
                               Point end_effector_vect, float z) {
  Point left_joint = get_joint(true, midpoint_distance, transmission_length,
                               proximal_length, base, left_slider);
  //std::cout << "left joint" << std::endl;
  //left_joint.print();
  Point right_joint = get_joint(false, midpoint_distance, transmission_length,
                                proximal_length, base, right_slider);
  //std::cout << "right joint" << std::endl;
  //right_joint.print();
  float h = std::sqrt(pow(distal_length, 2) -
                      pow((right_joint - left_joint).magnitude(), 2) / 4);
  if (is_upper) {
    h += end_effector_vect.magnitude();
  }
  
  //std::cout << "end effector values midpoint" << std::endl;
  //std::cout << "h " << h << std::endl;
  Point midpoint = (right_joint - left_joint) * 0.5;
  //midpoint.print();
  Point perpendicular_vector = {.x = -midpoint.y, .y = midpoint.x, .z = 0};
  //perpendicular_vector.print();
  Point resized_perp_vec = perpendicular_vector.normalize() * h;
  //resized_perp_vec.print();
  Point end_effector = (left_joint + right_joint)*0.5 + resized_perp_vec;
  //end_effector.print();
  if (!is_upper) {
    end_effector = ((end_effector - left_joint).normalize() *
                   (distal_length + end_effector_vect.magnitude())) + left_joint;
  }
  //end_effector.print();
  if (is_upper) {
    end_effector.z = UPPER_LINKAGE_Z;
  } else {
    end_effector.z = LOWER_LINKAGE_Z;
  }
  return end_effector;
}

Point get_needle_point_based_on_end_effector_positions(
    Point upper_linkage_end_effector, Point lower_linkage_end_effector,
    float needle_extension) {
  Point z_prime =
      (upper_linkage_end_effector - lower_linkage_end_effector).normalize();
  Point y_prime = cross(z_prime, {1, 0, 0}).normalize();
  Point x_prime = cross(z_prime, y_prime);
  z_prime = z_prime * (LOWER_END_EFFECTOR_TO_NEEDLEPOINT.z - needle_extension);
  y_prime = y_prime * LOWER_END_EFFECTOR_TO_NEEDLEPOINT.y;
  x_prime = x_prime * LOWER_END_EFFECTOR_TO_NEEDLEPOINT.x;

  return lower_linkage_end_effector + z_prime + y_prime + x_prime;
}

Point get_end_effector(Point left, Point left_middle, Point right_middle,
                       Point right, Point top_base, Point bottom_base,
                       float needle_extension) {
  // Z coordinates will be determined in post to make calculations easier, as
  // the z component is static and pre-determined.
  assert(left.z == 0 && right.z == 0 && right_middle.z == 0 &&
         left_middle.z == 0 && top_base.z == 0 && bottom_base.z == 0);
  Point upper_linkage_end_effector = get_linkage_end_effector(
      true, left, right, top_base, UPPER_TRANSMISSION_LENGTH,
      UPPER_PROXIMAL_LENGTH, UPPER_DISTAL_LENGTH, UPPER_MIDPOINT_DISTANCE,
      {0, 19.33, 0}, UPPER_LINKAGE_Z);
  //std::cout << "upper linkage end effector" << std::endl;
  //upper_linkage_end_effector.print();
  Point lower_linkage_end_effector = get_linkage_end_effector(
      false, left_middle, right_middle, bottom_base, LOWER_TRANSMISSION_LENGTH,
      LOWER_PROXIMAL_LENGTH, LOWER_DISTAL_LENGTH, LOWER_MIDPOINT_DISTANCE,
      {14, 0, 0}, LOWER_LINKAGE_Z);
  //std::cout << "lower linkage end effector" << std::endl;
  //lower_linkage_end_effector.print();
  return get_needle_point_based_on_end_effector_positions(
      upper_linkage_end_effector, lower_linkage_end_effector, needle_extension);
}
