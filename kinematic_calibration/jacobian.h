#ifndef JACOBIAN
#define JACOBIAN
#include "kinematics.h"
#include "templated_classes/Templated_Point.h"
#include "templated_classes/Templated_Transform.h"
#include <strings.h>
#include <vector>

/**
 * @brief Get the F_M1R base transform
 *
 * @tparam T 
 * @return The transform of the baseplate to the robot frame
 */
template <typename T> Transform<T> F_M1R();

/**
 * @brief Get the F_M2N transform, which returns the needle to marker transform, allowing pose measurement
 *
 * @tparam T 
 * @param theta_5 
 * @param parameters 
 * @return 
 */
template <typename T> Transform<T> F_M2N(T theta_5, Parameters<T> parameters);

/**
 * @brief Get the loop closure error of a position based on a measurement. 
 *
 * @tparam T 
 * @param thetas The position of the robot's sliders
 * @param parameters The kinematic parameters of the robot
 * @param measurement The measurement of the position of the markers
 * @param loop the kinematic loop we use to determine error
 * @return The error magnitude
 */
template <typename T>
T error(Thetas<T> thetas, Parameters<T> parameters, Measurement<T> measurement,
        linkage_values loop);

Parameters<double> ceres_solve(std::vector<Measurement<double>> measurements,
                               std::vector<Thetas<double>> thetas,
                               Parameters<double> guess);

/**
 * @brief Turn an atracsys measurement into a templated class
 *
 * @tparam T 
 * @param m the measurement
 * @return the templated measurement
 */
template <typename T> Measurement<T> measurement_to_T(Measurement<double> &m) {
  Measurement<T> ret;
  ret.F_OM1 = T(m.F_OM1.p);
}

/**
 * @struct Error_Residual
 * @brief This error residual struct is used to calculate residuals in order to conduct the kinematic calibration.
 *
 */
struct Error_Residual {
  Measurement<double> measurement;
  Thetas<double> thetas;
  linkage_values val;
  int index;
  Error_Residual(Measurement<double> m, Thetas<double> t, linkage_values v,
                 int i)
      : measurement(m), thetas(t), val(v), index(i) {}

  template <typename T>
  bool operator()(const T *const params, T *residual) const {
    Parameters<T> paramaters = array_to_Parameters<T>(params);
    Measurement<T> measurement_T = measurement.template convert<T>();
    Thetas<T> thetas_T = thetas.template convert<T>();
    T e = error(thetas_T, paramaters, measurement_T, val);
    residual[0] = e;
    return true;
  }
};

#endif // JACOBIAN
