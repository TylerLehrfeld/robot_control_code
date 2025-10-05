#include "jacobian.h"
#include "kinematics.h"
#include <cassert>
#include <ceres/jet.h>
#include <chrono>
#include <cmath>
#include <random>
#include <stdexcept>
#include <vector>
// We bring this in for our initial guess because I haven't implemented
// templated inverse_kinematics yet
namespace External {
#include "../inverse_kinematics.cpp"
}

Thetas<double> to_thetas(External::slider_positions s) {

  Thetas<double> t;
  t.theta_1 = s.left_slider_y;
  t.theta_2 = s.right_slider_y;
  t.theta_3 = s.left_middle_slider_y;
  t.theta_4 = s.right_middle_slider_y;
  t.theta_5 = s.needle_extension;
  return t;
}

int get_positions(std::vector<Thetas<double>> &positions) {
  External::Robot robot;
  int X_WIDTH = 100;
  int Y_HEIGHT = 500;
  int Y_MIN = 300;
  for (float x = -X_WIDTH; x <= X_WIDTH; x += 10) {
    for (float y = Y_MIN; y < Y_HEIGHT; y += 10) {
      for (int i = 0; i < 5; i++) {
        float theta;
        float phi = 0;
        if (i == 0) {
          theta = 0;
        } else {
          theta = M_PI / 4;
          phi += M_PI / 2;
        }
        External::approach_definition def = {{x, y, -100}, theta, phi};
        try {
          External::slider_positions sliders = External::inverse_kinematics(
              def, External::NewTransform(0, 0, 0, 0, 0, 0), robot);
          positions.push_back(to_thetas(sliders));
        } catch (std::runtime_error e) {
        }
      }
    }
  }
  return positions.size();
}



void get_simulated_measurements(std::vector<Measurement<double>> &measurements,
                                std::vector<Thetas<double>> &thetas,
                                Parameters<double> params) {
  for (int i = 0; i < thetas.size(); ++i) {
    Transform<double> I = Transform<double>(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    Transform<double> f_M1R = F_M1R<double>();
    Transform<double> ee = get_end_effector(thetas[i], params);
    Transform<double> f_M2N = F_M2N<double>(thetas[i].theta_5, params);
    // changed to identity transforms for simplicity.
    // Measurement<double> m = {I, f_M1R * ee * f_M2N.inverse()};
    Measurement<double> m = {I, ee};
    measurements.push_back(m);
  }
}

void compare_params(Parameters<double> actual, Parameters<double> optimized,
                    const int ct) {
  double *arr_1 = new double[ct];
  int count = Parameters_to_array<double>(actual, arr_1);
  assert(count == ct);
  double *arr_2 = new double[ct];
  Parameters_to_array<double>(optimized, arr_2);
  double mse_tot = 0;
  for (int i = 0; i < count; i++) {
    double diff = arr_1[i] - arr_2[i];
    std::cout << "difference in parameter " << i << " = " << diff << std::endl;
    mse_tot += diff * diff;
  }
  mse_tot /= count;
  std::cout << "Mean squared error = " << mse_tot << std::endl;
  delete[] arr_2;
  delete[] arr_1;
}

double get_error_cost(std::vector<Measurement<double>> &measurements,
                      std::vector<Thetas<double>> &thetas,
                      Parameters<double> guess) {
  double mse = 0;
  for (int i = 0; i < measurements.size(); ++i) {
    for (int j = 0; j < num_loops; ++j) {
      double err = error<double>(thetas[i], guess, measurements[i],
                                 static_cast<linkage_values>(j));
      mse += err * err;
    }
  }
  return mse / 2;
}

int main(int argc, char *argv[]) {
  constexpr Parameters<double> guess = get_default_parameters<double>();
  constexpr int ct = get_num_tunable_params(guess);
  Parameters<double> actual = adjust_params(guess);
  compare_params(actual, guess, ct);
  std::vector<Thetas<double>> thetas;
  std::vector<Measurement<double>> measurements;
  get_positions(thetas);
  get_simulated_measurements(measurements, thetas, actual);
  double unoptimized_mse = get_error_cost(measurements, thetas, guess);
  double minimal_mse = get_error_cost(measurements, thetas, actual);
  std::cout << "unoptimized_cost: " << unoptimized_mse
            << " minimal_cost(should be zero): " << minimal_mse << std::endl;
  Parameters<double> optimized = ceres_solve<ct>(measurements, thetas, guess);
  double optimized_mse = get_error_cost(measurements, thetas, optimized);

  std::cout << "optimized_cost: " << optimized_mse << std::endl;
  compare_params(actual, optimized, ct);
}
