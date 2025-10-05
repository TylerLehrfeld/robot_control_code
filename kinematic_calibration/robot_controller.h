#include "../galil_control_calls.h"
#include "kinematics.h"

const static double theta_1_backlash = .02;
const static double theta_2_backlash = .02;
const static double theta_3_backlash = .02;
const static double theta_4_backlash = .02;

class RobotController {
public:
  bool compensating;
  Parameters<double> parameters;
  bool left_forward, right_forward, left_middle_forward,
      right_middle_forward = true;
  Thetas<double> cur_thetas = home_positions;
  RobotController(bool compensation, Parameters<double> parameters)
      : compensating(compensation), parameters(parameters) {
    left_forward = right_middle_forward = right_forward = left_middle_forward =
        true;
  }

  RobotController(bool compensation) : compensating(compensation) {
    left_forward = right_middle_forward = right_forward = left_middle_forward =
        true;
    Parameters<double> p = get_default_parameters<double>();
  }
  void home() {

    bool left_behind, right_behind, left_middle_behind, right_middle_behind;
    left_behind = right_behind = left_middle_behind = right_middle_behind =
        false;
    std::string input_str;
    std::cout << "is the left slider behind the flag?" << std::endl;
    std::cin >> input_str;
    if (input_str == "y" || input_str == "yes") {
      left_behind = true;
    }
    std::cout << "is the right slider behind the flag?" << std::endl;
    std::cin >> input_str;

    if (input_str == "y" || input_str == "yes") {
      right_behind = true;
    }
    std::cout << "is the left middle slider behind the flag?" << std::endl;
    std::cin >> input_str;

    if (input_str == "y" || input_str == "yes") {
      left_middle_behind = true;
    }
    std::cout << "is the right middle slider behind the flag?" << std::endl;
    std::cin >> input_str;

    if (input_str == "y" || input_str == "yes") {
      right_middle_behind = true;
    }
    init_galil(0);
    std::cout << "Ready to home Low?" << std::endl;
    std::cin >> input_str;
    HomeLowBlocking(right_middle_behind, left_middle_behind);
    std::cout << "Ready to home Up?" << std::endl;
    std::cin >> input_str;
    HomeUpBlocking(right_behind, left_behind);
    std::cout << "Done Home" << std::endl;
  }
  encoder_error_struct move(Thetas<double> new_thetas) {
    init_galil(5);
    Thetas<double> targeting_thetas = new_thetas;
    if (compensating) {
      // left slider
      if (left_forward) {
        if (new_thetas.theta_1 < cur_thetas.theta_1) {
          targeting_thetas.theta_1 -= theta_1_backlash;
          left_forward = false;
        }
      } else {
        if (new_thetas.theta_1 > cur_thetas.theta_1) {
          targeting_thetas.theta_1 += theta_1_backlash;
          left_forward = true;
        }
      } // left slider end
      // right slider
      if (right_forward) {
        if (new_thetas.theta_2 < cur_thetas.theta_2) {
          targeting_thetas.theta_2 -= theta_2_backlash;
          right_forward = false;
        }
      } else {
        if (new_thetas.theta_2 > cur_thetas.theta_2) {
          targeting_thetas.theta_2 += theta_2_backlash;
          right_forward = true;
        }
      } // right slider end
      // left middle slider
      if (left_middle_forward) {
        if (new_thetas.theta_3 < cur_thetas.theta_3) {
          targeting_thetas.theta_3 -= theta_3_backlash;
          left_middle_forward = false;
        }
      } else {
        if (new_thetas.theta_3 > cur_thetas.theta_3) {
          targeting_thetas.theta_3 += theta_3_backlash;
          left_middle_forward = true;
        }
      } // left middle slider end
        // right middle slider
      if (right_middle_forward) {
        if (new_thetas.theta_4 < cur_thetas.theta_4) {
          targeting_thetas.theta_4 -= theta_4_backlash;
          right_middle_forward = false;
        }
      } else {
        if (new_thetas.theta_4 > cur_thetas.theta_4) {
          targeting_thetas.theta_4 += theta_4_backlash;
          right_middle_forward = true;
        }
      } // right middle slider end
    }
    encoder_error_struct errs =
        move_robot_with_slider_positions(targeting_thetas);
    cur_thetas = new_thetas;
    cur_thetas.theta_1 += errs.mmErrLeft;
    cur_thetas.theta_2 += errs.mmErrRight;
    cur_thetas.theta_3 += errs.mmErrLeftMiddle;
    cur_thetas.theta_4 += errs.mmErrRightMiddle;
    return errs;
  }
};
