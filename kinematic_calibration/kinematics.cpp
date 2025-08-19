#include "kinematics.h"
#include "templated_classes/Templated_Point.h"
#include "templated_classes/Templated_Transform.h"
#include <cmath>

template <typename T>
Transform<T> get_end_effector(Thetas<T> thetas, Parameters<T> parameters) {
  Point<T> E1 = get_upper_linkage_E<T>(thetas, parameters);
  Point<T> E2 = get_lower_linkage_E<T>(thetas, parameters);
  Point<T> diff = (E2 - E1).normalize();
  Point<T> N =
      E2 + (parameters.tunable_params.needle_offset + thetas.theta_5) * (diff);
  Point<T> n_vec = get_upper_linkage_n_vec<T>(thetas, parameters);
  Point<T> z = -diff;
  Point<T> y = cross(n_vec, z).normalize();
  Point<T> x = cross(y, z);
  Transform<T> EE_frame(x, y, z, N);
  return EE_frame;
}

template <typename T>
Point<T> get_upper_linkage_E(Thetas<T> thetas, Parameters<T> parameters) {
  Point<T> P1 = get_upper_linkage_P(thetas, parameters);
  Point<T> P2 = get_lower_linkageP(thetas, parameters);
  Point<T> diff = (P2 - P1);
  Point<T> z = {T(0), T(0), T(1)};
  Point<T> n_vec = get_upper_linkage_n_vec<T>(thetas, parameters);
  T e1 = parameters.tunable_params.top_needle_to_holder_distance;
  return P1 +
         e1 * (-(diff * z) * n_vec + (diff * n_vec) * z) / diff.magnitude();
}

template <typename T>
Point<T> get_lower_linkage_E(Thetas<T> thetas, Parameters<T> parameters) {
  Point<T> P1 = get_upper_linkage_P(thetas, parameters);
  Point<T> P2 = get_lower_linkage_P(thetas, parameters);
  Point<T> diff = (P2 - P1);
  Point<T> z = {T(0), T(0), T(1)};
  Point<T> n_vec = get_upper_linkage_n_vec<T>(thetas, parameters);
  T e2 = parameters.tunable_params.bottom_needle_to_holder_distance;
  return P2 +
         e2 * (-(diff * z) * n_vec + (diff * n_vec) * z) / diff.magnitude();
}

template <typename T>
Point<T> get_upper_linkage_n_vec(Thetas<T> thetas, Parameters<T> parameters) {
  Point<T> upper_C = get_linkage_C(thetas, parameters, TOP_LEFT);
  Point<T> D = get_D(thetas, parameters);
  return (upper_C - D).normalize();
}

template <typename T>
Point<T> get_D(Thetas<T> thetas, Parameters<T> parameters) {
  Point upper_C =
      get_linkage_C(thetas.theta_1, thetas.theta_2,
                    parameters.tunable_params.loop_parameters[TOP_LEFT],
                    parameters.tunable_params.loop_parameters[TOP_RIGHT]);
  T ratio1 =
      parameters.tunable_params.D1 /
      parameters.tunable_params.loop_parameters[TOP_LEFT].distal_link_length;
  T ratio2 =
      parameters.tunable_params.D2 /
      parameters.tunable_params.loop_parameters[TOP_RIGHT].distal_link_length;
  Point<T> B_1 = get_linkage_B(thetas, parameters, TOP_LEFT);
  Point<T> B_2 = get_linkage_B(thetas, parameters, TOP_RIGHT);
  Point<T> Q1 = (1 - ratio1) * upper_C + ratio1 * B_1;
  Point<T> Q2 = (1 - ratio2) * upper_C + ratio2 * B_2;
  return third_point_in_triangle(Q2, Q1, parameters.tunable_params.D3,
                                 parameters.tunable_params.D4);
}

template <typename T>
Point<T> get_lower_linkage_P(Thetas<T> thetas, Parameters<T> parameters) {
  Point<T> lower_C = get_linkage_C(thetas, parameters);
  Point<T> lower_left_B = get_linkage_B(thetas, parameters, BOTTOM_LEFT);
  Point<T> P =
      lower_C + ((lower_C - lower_left_B) *
                 (parameters.tunable_params.loop_parameters[BOTTOM_LEFT] +
                  parameters.tunable_params.bottom_holder_to_linkage_distance) /
                 parameters.tunable_params.loop_parameters[BOTTOM_LEFT]);
  return P;
}

template <typename T>
Point<T> get_upper_linkage_P(Thetas<T> thetas, Parameters<T> parameters) {
  Point<T> upper_C = get_linkage_C(thetas, parameters, TOP_RIGHT);
  Point<T> D = get_D(thetas, parameters);
  T n1 = parameters.tunable_params.top_holder_to_linkage_distance;
  return upper_C + (upper_C - D) * n1 / (upper_C - D).magnitude();
}

template <typename T>
Point<T> get_linkage_C(Thetas<T> thetas, Parameters<T> parameters,
                       linkage_values val) {
  Point<T> B_j;
  Point<T> B_k;
  T d_j;
  T d_k;
  if (val == TOP_RIGHT || val == TOP_LEFT) {
    B_j = get_linkage_B(thetas, parameters, TOP_LEFT);
    B_k = get_linkage_B(thetas, parameters, TOP_RIGHT);
    d_j =
        parameters.tunable_params.loop_parameters[TOP_LEFT].distal_link_length;
    d_k =
        parameters.tunable_params.loop_parameters[TOP_RIGHT].distal_link_length;
  } else if (val == BOTTOM_LEFT || val == BOTTOM_RIGHT) {
    B_j = get_linkage_B(thetas, parameters, BOTTOM_LEFT);
    B_k = get_linkage_B(thetas, parameters, BOTTOM_RIGHT);
    d_j = parameters.tunable_params.loop_parameters[BOTTOM_LEFT]
              .distal_link_length;
    d_k = parameters.tunable_params.loop_parameters[BOTTOM_RIGHT]
              .distal_link_length;
  }
  return third_point_in_triangle(B_j, B_k, d_k, d_j);
}

template <typename T>
Point<T> get_linkage_B(Thetas<T> thetas, Parameters<T> parameters,
                       linkage_values val) {
  Point<T> base;
  Point<T> S_i;
  if (val == TOP_RIGHT || val == TOP_LEFT) {
    base = parameters.upper_base;
    base.z += parameters.tunable_params.upper_base_z_offset;
    if (val == TOP_RIGHT) {
      S_i = {
          parameters.tunable_params.loop_parameters[TOP_RIGHT].x_slider_offset,
          thetas.theta_2 + parameters.tunable_params.loop_parameters[TOP_RIGHT]
                               .y_slider_offset,
          base.z};
    } else if (val == TOP_LEFT) {
      S_i = {
          parameters.tunable_params.loop_parameters[TOP_LEFT].x_slider_offset,
          thetas.theta_1 + parameters.tunable_params.loop_parameters[TOP_LEFT]
                               .y_slider_offset,
          base.z};
    }
  } else if (val == BOTTOM_LEFT || val == BOTTOM_RIGHT) {
    base = parameters.lower_base;
    base.z += parameters.tunable_params.lower_base_z_offset;
    if (val == BOTTOM_LEFT) {
      S_i = {parameters.tunable_params.loop_parameters[BOTTOM_LEFT]
                 .x_slider_offset,
             thetas.theta_3 +
                 parameters.tunable_params.loop_parameters[BOTTOM_LEFT]
                     .y_slider_offset,
             base.z};
    } else if (val == BOTTOM_RIGHT) {
      S_i = {parameters.tunable_params.loop_parameters[BOTTOM_RIGHT]
                 .x_slider_offset,
             thetas.theta_1 +
                 parameters.tunable_params.loop_parameters[BOTTOM_RIGHT]
                     .y_slider_offset,
             base.z};
    }
  }
  T proximal_link_length =
      parameters.tunable_params.loop_parameters[val].proximal_link_length;
  T proximal_link_midpoint =
      parameters.tunable_params.loop_parameters[val].proximal_link_midpoint;
  T transmission_link_length =
      parameters.tunable_params.loop_parameters[val].transmission_link_length;
  Point<T> M;
  if (val == BOTTOM_LEFT || val == TOP_LEFT) {
    M = third_point_in_triangle(S_i, base, proximal_link_midpoint,
                                transmission_link_length);
  } else if (val == BOTTOM_RIGHT || val == TOP_RIGHT) {
    M = third_point_in_triangle(base, S_i, transmission_link_length,
                                proximal_link_midpoint);
  }
  T ratio = proximal_link_length / proximal_link_midpoint;
  Point<T> B_i = ratio * (M - base) + base;
  return B_i;
}
// Thetas inverse_kinematics(Parameters parameters, NewTransform T) {
//	parameters.upper_base.z +=
// parameters.tunable_params.upper_base_z_offset; 	parameters.lower_base.z
// += parameters.tunable_params.lower_base_z_offset; 	Thetas t; 	Point
// upper_P = T.p(); //TODO 	Point lower_P; //TODO
// update_upper_linkage_thetas(parameters, upper_P, t);
// update_lower_linkage_thetas(parameters, lower_P, t); 	return t;
// }
