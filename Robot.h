#ifndef ROBOT
#define ROBOT

#include "kinematic_structs.h"

class Robot {
public:
  Point origin = {0, 0, 0};
  Point needle_tip = {0, 0, 0};
  Point x_prime = {0, 0, 0};
  Point y_prime = {0, 0, 0};
  Point z_prime = {0, 0, 0};
  linkage_array top_linkage = {
      .extended_end_effector = {0, 0, 0},
      .linkage_end_effector = {0, 0, 0},
      .base = UPPER_BASE,
      .left_joint = {0, 0, 0},
      .right_joint = {0, 0, 0},
      .left_midpoint = {0, 0, 0},
      .right_midpoint = {0, 0, 0},
  };
  linkage_array bottom_linkage = {
      .extended_end_effector = {0, 0, 0},
      .linkage_end_effector = {0, 0, 0},
      .base = LOWER_BASE,
      .left_joint = {0, 0, 0},
      .right_joint = {0, 0, 0},
      .left_midpoint = {0, 0, 0},
      .right_midpoint = {0, 0, 0},
  };
  slider_positions sliders = {
      .left_slider_y = 0,
      .left_middle_slider_y = 0,
      .right_middle_slider_y = 0,
      .right_slider_y = 0,
      .needle_extension = 0,
  };
  
};

#endif