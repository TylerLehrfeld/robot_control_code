#include "../forward_kinematics.h"
#include "../inverse_kinematics.h"
#include "../robot_controller.h"
#include "kinematics.h"
#include <vector>
#define pi 3.141592653

int get_positions(std::vector<slider_positions> &positions) {
  Robot robot;
  int X_WIDTH = 200;
  int Y_HEIGHT = 500;
  for (float x = -X_WIDTH; x <= X_WIDTH; x += 10) {
    for (float y = 0; y < Y_HEIGHT; y += 10) {
      for (int i = 0; i < 5; i++) {
        float theta;
        float phi = 0;
        if (i == 0) {
          theta = 0;
        } else {
          theta = pi / 4;
          phi += pi / 2;
        }
        approach_definition def = {{x, y, -50}, theta, phi};
        positions.push_back(
            inverse_kinematics(def, NewTransform(0, 0, 0, 0, 0, 0), robot));
      }
    }
  }
  return positions.size();
}

Parameters get_default_params() {
  // TODO
}

Tunable_parameters get_best_params(vector<Measurement> &measurement_array,
                                   vector<slider_positions> positions,
                                   NewTransform F_MR, NewTransform F_MN,
                                   int NR_max_iters = 750) {
  Parameters parameters = get_default_params();
  for (int NR_iter = 0; NR_iter < NR_max_iters; NR_iter++) {
    // loop over each position

    for (Measurement measurement : measurement_array) {
      // loop over each linkage loop
      for (int i = 0; i < 4; i++) {
        // calculate f_i,j
      }
    }
    Matrix J();
    w = update_w(w, J_M);
  }
}

static const slider_positions home_positions = {
    BASE_TO_SLIDER_MAX - 60,    //- HALF_SLIDER_WIDTH,
    BASE_TO_SLIDER_MAX - 26.79, // - HALF_SLIDER_WIDTH,
    BASE_TO_SLIDER_MAX - 23.48, //- HALF_SLIDER_WIDTH,
    BASE_TO_SLIDER_MAX - 60,    //- HALF_SLIDER_WIDTH,
    0};
static const NewTransform F_MR(0, 0, 0, 0, 0, 0);
static const NewTransform F_MN(0, 0, 0, 0, 0, 0);
int main() {
  vector<slider_positions> positions;
  int num_positions = get_positions(positions);
  robot_controller rc(home_positions);
  vector<Measurement> measurement_array;
  for (slider_positions position : positions) {
    // go to position
    // read values
    // write values to file
  }

  get_best_params(measurement_array, positions, F_MR, F_MN);
}
