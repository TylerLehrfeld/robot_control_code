#ifndef CAL_KINEMATICS
#define CAL_KINEMATICS

#include "./templated_classes/Templated_Point.h"
#include "./templated_classes/Templated_Transform.h"
#include <stdexcept>
#include <vector>

#define num_loops 4

enum linkage_values {
  TOP_LEFT = 0,
  TOP_RIGHT = 1,
  BOTTOM_LEFT = 2,
  BOTTOM_RIGHT = 3,
};

template <typename T> struct Measurement {
  Transform<T> F_OM1;
  Transform<T> F_OM2;
  template <typename U> Measurement<U> convert() const {
    Measurement<U> M;
    M.F_OM1 = F_OM1.template convert<U>();
    M.F_OM2 = F_OM2.template convert<U>();
    return M;
  }
};

template <typename T> struct Thetas {
  T theta_1;
  T theta_2;
  T theta_3;
  T theta_4;
  T theta_5;
  template <typename U> Thetas<U> convert() const {
    Thetas<U> thetas;
    thetas.theta_1 = U(theta_1);
    thetas.theta_2 = U(theta_2);
    thetas.theta_3 = U(theta_3);
    thetas.theta_4 = U(theta_4);
    thetas.theta_5 = U(theta_5);
    return thetas;
  }
};

template <typename T> struct Loop_parameters {
  T x_slider_offset;
  T y_slider_offset;
  T transmission_link_length;
  T proximal_link_length;
  T proximal_link_midpoint;
  T distal_link_length;
};

template <typename T> struct Tunable_parameters {

  /**
   * 0: top left
   * 1: top right
   * 2: bottom left
   * 3: bottom right
   **/
  Loop_parameters<T> loop_parameters[num_loops];
  T top_needle_to_holder_distance;
  T bottom_needle_to_holder_distance;
  T top_holder_to_linkage_distance;
  T bottom_holder_to_linkage_distance;
  T needle_offset;
  T lower_base_z_offset;
  T upper_base_z_offset;
  T D1;
  T D2;
  T D3;
  T D4;
};

template <typename T> struct Parameters {
  inline static int num_tunamble_params;
  Tunable_parameters<T> tunable_params;
  Point<T> upper_base;
  Point<T> lower_base;
};

template <typename T> Parameters<T> get_default_parameters() {
  Parameters<T> params;
  T sliderXs[num_loops] = {T(-63), T(63), T(-21), T(21)};
  for (int i = 0; i < num_loops; i++) {
    params.tunable_params.loop_parameters[i].x_slider_offset = sliderXs[i];
    params.tunable_params.loop_parameters[i].y_slider_offset = T(0);
    params.tunable_params.loop_parameters[i].transmission_link_length =
        i < 2 ? T(115) : T(110);
    params.tunable_params.loop_parameters[i].proximal_link_midpoint =
        i < 2 ? T(58) : T(46);
    params.tunable_params.loop_parameters[i].proximal_link_length =
        i < 2 ? T(116) : T(92);
    params.tunable_params.loop_parameters[i].distal_link_length =
        i < 2 ? T(125) : T(99);
  }
  params.tunable_params.top_needle_to_holder_distance = T(47.7);
  params.tunable_params.bottom_needle_to_holder_distance = T(47.7);
  params.tunable_params.top_holder_to_linkage_distance = T(19.25);
  params.tunable_params.bottom_holder_to_linkage_distance = T(14);
  params.tunable_params.needle_offset = T(64.9);
  params.tunable_params.lower_base_z_offset = T(0);
  params.tunable_params.upper_base_z_offset = T(31.96);
  params.tunable_params.D1 = T(62.5);
  params.tunable_params.D2 = T(62.5);
  params.tunable_params.D3 = T(62.5);
  params.tunable_params.D4 = T(62.5);
  params.lower_base = {T(0), T(224), T(0)};
  params.upper_base = {T(0), T(186), T(0)};
  return params;
}

template <typename T> inline T *Parameters_to_array(Parameters<T> parameters) {
  T *array = new T[parameters.num_tunamble_params];
  Tunable_parameters<T> tunables = parameters.tunable_params;
  for (int i = 0; i < num_loops; ++i) {
    array[i * 6 + 0] = tunables.loop_parameters[i].x_slider_offset;
    array[i * 6 + 1] = tunables.loop_parameters[i].y_slider_offset;
    array[i * 6 + 2] = tunables.loop_parameters[i].transmission_link_length;
    array[i * 6 + 3] = tunables.loop_parameters[i].proximal_link_length;
    array[i * 6 + 4] = tunables.loop_parameters[i].proximal_link_midpoint;
    array[i * 6 + 5] = tunables.loop_parameters[i].distal_link_length;
  }
  array[24] = tunables.top_holder_to_linkage_distance;
  array[25] = tunables.bottom_holder_to_linkage_distance;
  array[26] = tunables.needle_offset;
  array[27] = tunables.lower_base_z_offset;
  array[28] = tunables.upper_base_z_offset;
  array[29] = tunables.top_needle_to_holder_distance;
  array[30] = tunables.bottom_needle_to_holder_distance;
  array[31] = tunables.D1;
  array[32] = tunables.D2;
  array[33] = tunables.D3;
  array[34] = tunables.D4;
  return array;
}

template <typename T>
inline Parameters<T> array_to_Parameters(T const *tunable_params) {
  Parameters<T> p = get_default_parameters<T>();
  for (int i = 0; i < num_loops; i++) {
    p.tunable_params.loop_parameters[i].x_slider_offset =
        tunable_params[i * 6 + 0];
    p.tunable_params.loop_parameters[i].y_slider_offset =
        tunable_params[i * 6 + 1];
    p.tunable_params.loop_parameters[i].transmission_link_length =
        tunable_params[i * 6 + 2];
    p.tunable_params.loop_parameters[i].proximal_link_length =
        tunable_params[i * 6 + 3];
    p.tunable_params.loop_parameters[i].proximal_link_midpoint =
        tunable_params[i * 6 + 4];
    p.tunable_params.loop_parameters[i].distal_link_length =
        tunable_params[i * 6 + 5];
  }
  p.tunable_params.top_holder_to_linkage_distance = tunable_params[24];
  p.tunable_params.bottom_holder_to_linkage_distance = tunable_params[25];
  p.tunable_params.needle_offset = tunable_params[26];
  p.tunable_params.lower_base_z_offset = tunable_params[27];
  p.tunable_params.upper_base_z_offset = tunable_params[28];
  p.tunable_params.top_needle_to_holder_distance = tunable_params[29];
  p.tunable_params.bottom_needle_to_holder_distance = tunable_params[30];
  p.tunable_params.D1 = tunable_params[31];
  p.tunable_params.D2 = tunable_params[32];
  p.tunable_params.D3 = tunable_params[33];
  p.tunable_params.D4 = tunable_params[34];
  return p;
}

template <typename T>
Point<T> get_upper_linkage_n_vec(const Thetas<T> &thetas,
                                 const Parameters<T> &parameters) {
  Point<T> upper_C = get_linkage_C(thetas, parameters, TOP_LEFT);
  Point<T> D = get_D(thetas, parameters);
  return (upper_C - D).normalize();
}

template <typename T>
Point<T> get_upper_linkage_E(const Thetas<T> &thetas,
                             const Parameters<T> &parameters) {
  Point<T> P1 = get_upper_linkage_P(thetas, parameters);
  Point<T> P2 = get_lower_linkage_P(thetas, parameters);
  Point<T> diff = (P2 - P1);
  Point<T> z = {T(0), T(0), T(1)};
  Point<T> n_vec = get_upper_linkage_n_vec<T>(thetas, parameters);
  T e1 = parameters.tunable_params.top_needle_to_holder_distance;
  return P1 + e1 * ((-(diff * z) * n_vec) + ((diff * n_vec) * z)) *
                  (T(1) / diff.magnitude());
}

template <typename T>
Point<T> get_lower_linkage_E(const Thetas<T> &thetas,
                             const Parameters<T> &parameters) {
  Point<T> P1 = get_upper_linkage_P(thetas, parameters);
  Point<T> P2 = get_lower_linkage_P(thetas, parameters);
  Point<T> diff = (P2 - P1);
  Point<T> z = {T(0), T(0), T(1)};
  Point<T> n_vec = get_upper_linkage_n_vec<T>(thetas, parameters);
  T e2 = parameters.tunable_params.bottom_needle_to_holder_distance;
  return P2 + e2 * ((-(diff * z) * n_vec) + ((diff * n_vec) * z)) *
                  (T(1) / diff.magnitude());
}

template <typename T>
Transform<T> get_end_effector(const Thetas<T> &thetas,
                              const Parameters<T> &parameters) {
  Point<T> E1 = get_upper_linkage_E<T>(thetas, parameters);
  Point<T> E2 = get_lower_linkage_E<T>(thetas, parameters);
  Point<T> diff = (E2 - E1).normalize();
  Point<T> N =
      E2 + (parameters.tunable_params.needle_offset + thetas.theta_5) * (diff);
  Point<T> n_vec = get_upper_linkage_n_vec<T>(thetas, parameters);
  Point<T> z = T(-1) * diff;
  Point<T> y = cross(n_vec, z).normalize();
  Point<T> x = cross(y, z);
  Transform<T> EE_frame(x, y, z, N);
  return EE_frame;
}

template <typename T>
Point<T> get_D(const Thetas<T> &thetas, const Parameters<T> &parameters) {
  Point upper_C = get_linkage_C(thetas, parameters, TOP_LEFT);
  T ratio1 =
      parameters.tunable_params.D1 /
      parameters.tunable_params.loop_parameters[TOP_LEFT].distal_link_length;
  T ratio2 =
      parameters.tunable_params.D2 /
      parameters.tunable_params.loop_parameters[TOP_RIGHT].distal_link_length;
  Point<T> B_1 = get_linkage_B(thetas, parameters, TOP_LEFT);
  Point<T> B_2 = get_linkage_B(thetas, parameters, TOP_RIGHT);
  Point<T> Q1 = (T(1) - ratio1) * upper_C + ratio1 * B_1;
  Point<T> Q2 = (T(1) - ratio2) * upper_C + ratio2 * B_2;
  return third_point_in_triangle(Q2, Q1, parameters.tunable_params.D3,
                                 parameters.tunable_params.D4);
}

template <typename T>
Point<T> get_upper_linkage_P(const Thetas<T> &thetas,
                             const Parameters<T> &parameters) {
  Point<T> upper_C = get_linkage_C(thetas, parameters, TOP_RIGHT);
  Point<T> D = get_D(thetas, parameters);
  T n1 = parameters.tunable_params.top_holder_to_linkage_distance;
  return upper_C + (upper_C - D) * (n1 / (upper_C - D).magnitude());
}

template <typename T>
Point<T> get_linkage_C(const Thetas<T> &thetas, const Parameters<T> &parameters,
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
Point<T> get_lower_linkage_P(const Thetas<T> &thetas,
                             const Parameters<T> &parameters) {
  Point<T> lower_C = get_linkage_C(thetas, parameters, BOTTOM_LEFT);
  Point<T> lower_left_B = get_linkage_B(thetas, parameters, BOTTOM_LEFT);
  Point<T> P =
      lower_C + ((lower_C - lower_left_B) *
                 (parameters.tunable_params.loop_parameters[BOTTOM_LEFT]
                      .distal_link_length +
                  parameters.tunable_params.bottom_holder_to_linkage_distance) *
                 (T(1) / parameters.tunable_params.loop_parameters[BOTTOM_LEFT]
                             .distal_link_length));
  return P;
}

template <typename T>
Point<T> get_linkage_B(const Thetas<T> &thetas, const Parameters<T> &parameters,
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
             thetas.theta_4 +
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
// template <typename T>
// inline Thetas<T> inverse_kinematics(Parameters<T> parameters, Transform<T>
// transform);

//     l1
//  P3____
// l2\    P2
//    \ t/ l3
//     P1
//
template <typename T>
inline Point<T> third_point_in_triangle(Point<T> P1, Point<T> P2, T l1, T l2) {
  T z = P1.z;
  P1.z = T(0);
  P2.z = T(0);
  Point<T> P3;
  Point<T> diff = P2 - P1;
  if (diff.magnitude() > l1 + l2 || l1 + diff.magnitude() < l2 ||
      l2 + diff.magnitude() < l1) {
    throw std::runtime_error("Points are too far apart to form triangle");
  }
  Point<T> norm = diff.normalize();

  // we will always rotate clockwize from diff.
  Point<T> rotated_norm = {-norm.y, norm.x, T(0)};
  T l3 = (P1 - P2).magnitude();

  T l2cos_t = (l2 * l2 + l3 * l3 - l1 * l1) / (T(2) * l3);
  T h = sqrt(l2 * l2 - l2cos_t * l2cos_t);
  P3 = P1 + (l2cos_t * norm) + (h * rotated_norm);
  P3.z = z;
  return P3;
};

int get_positions(std::vector<Thetas<double>> &positions);
#endif // CAL_KINEMATICS
