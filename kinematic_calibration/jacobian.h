#ifndef JACOBIAN
#define JACOBIAN
#include "kinematics.h"
#include "templated_classes/Templated_Point.h"
#include "templated_classes/Templated_Transform.h"
#include <vector>

template <typename T> Transform<T> F_M1R();

template <typename T> Transform<T> F_M2N(T theta_5, Parameters<T> parameters);

template <typename T>
T error(Thetas<T> thetas, Parameters<T> parameters, Measurement<T> measurement,
        linkage_values loop);

Parameters<double> ceres_solve(std::vector<Measurement<double>> measurements,
                               std::vector<Thetas<double>> thetas,
                               Parameters<double> guess);

template <typename T> Measurement<T> measurement_to_T(Measurement<double> &m) {
  Measurement<T> ret;
  ret.F_OM1 = T(m.F_OM1.p);
}

struct Error_Residual {
  Measurement<double> measurement;
  Thetas<double> thetas;
  linkage_values val;

  Error_Residual(Measurement<double> m, Thetas<double> t, linkage_values v)
      : measurement(m), thetas(t), val(v) {}

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
