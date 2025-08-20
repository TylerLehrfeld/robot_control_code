#include "jacobian.h"
#include "ceres/ceres.h"
#include "kinematics.h"
#include "templated_classes/Templated_Point.h"
#include <ceres/autodiff_cost_function.h>
#include <ceres/cost_function.h>
#include <ceres/types.h>

template <typename T> Transform<T> F_M1R() {
  return Transform<T>(T(0), T(0), T(0), T(0), T(0), T(0));
};
template <typename T> Transform<T> F_M2N(T theta_5, Parameters<T> parameters) {
  // Replace 0s pivot calibration
  Transform<T> pivot;
  return pivot * Transform<T>(
                     T(0), T(0), T(0), T(0),
                     parameters.tunable_params.bottom_needle_to_holder_distance,
                     parameters.tunable_params.needle_offset + theta_5);
};

template <typename T>
T error(Thetas<T> thetas, Parameters<T> parameters, Measurement<T> measurement,
        linkage_values loop) {
  Point<T> zero = {T(0), T(0), T(0)};
  F_M1R<T>();
  F_M2N(thetas.theta_5, parameters);
  //  Point<T> N_obs = F_M1R<T>().inverse() * measurement.F_OM1.inverse() *
  //                  measurement.F_OM2 * F_M2N(thetas.theta_5, parameters) *
  //                  zero;
  Point<T> N_obs = measurement.F_OM2.p;
  Point<T> N_calc = get_end_effector(thetas, parameters).p;
  Point<T> C_i = get_linkage_C(thetas, parameters, loop);
  Point<T> B_i = get_linkage_B(thetas, parameters, loop);
  T c_i = parameters.tunable_params.loop_parameters[loop].distal_link_length;
  return (N_obs - N_calc + C_i - B_i).magnitude() - c_i;
}

Parameters<double> ceres_solve(std::vector<Measurement<double>> measurements,
                               std::vector<Thetas<double>> thetas,
                               Parameters<double> guess) {
  ceres::Problem problem;
  double *param_arr = Parameters_to_array(guess);
  for (int i = 0; i < measurements.size(); i++) {
    for (int j = 0; j < num_loops; j++) {
      // minimize tunable parameters wrt error for each loop for each
      ceres::CostFunction *cf =
          new ceres::AutoDiffCostFunction<Error_Residual, 1, 35>(
              new Error_Residual(measurements[i], thetas[i],
                                 static_cast<linkage_values>(j), i));
      problem.AddResidualBlock(cf, nullptr, param_arr);
    }
  }
  for (int k = 0; k < 35; ++k) {
    problem.SetParameterLowerBound(param_arr, k, param_arr[k] - 0.1);
    problem.SetParameterUpperBound(param_arr, k, param_arr[k] + 0.1);
  }
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  options.minimizer_progress_to_stdout = true;
  options.num_threads = 20;
  options.max_num_iterations = 500;
  options.dense_linear_algebra_library_type = ceres::CUDA;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  std::cout << summary.BriefReport() << "\n";

  std::cout << "Ceres Initial cost: " << summary.initial_cost
            << ", Final cost: " << summary.final_cost << "\n";
  Parameters<double> optimized = array_to_Parameters(param_arr);
  delete[] param_arr;
  return optimized;
}
