#include "Robot.h"
#include "forward_kinematics.h"
#include "inverse_kinematics.h"

void update_robot_forward(slider_positions sliders, Robot& robot) {
    get_end_effector({sliderXs[0], sliders.left_slider_y, 0}, {sliderXs[1],sliders.right_middle_slider_y, 0},{sliderXs[2], sliders.right_middle_slider_y, 0}, {sliderXs[3], sliders.right_slider_y, 0}, UPPER_BASE, LOWER_BASE, sliders.needle_extension, robot);
}

void update_robot_backward(approach_definition approach_definition, Robot& robot) {
  inverse_kinematics(approach_definition, Transform(0, 0, 0, 0, 0, 0), robot);
}

int main() {
    approach_definition inverse_approach = {{0, 350, -75},
                             0,
                             0};
    Robot inverse_robot;
    update_robot_backward(inverse_approach, inverse_robot);
    Robot forward_robot;
    update_robot_forward(inverse_robot.sliders, forward_robot);

}