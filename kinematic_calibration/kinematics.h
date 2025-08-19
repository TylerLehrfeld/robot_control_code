#ifndef CAL_KINEMATICS
#define CAL_KINEMATICS

#include "./templated_classes/Templated_Point.h"
#include "./templated_classes/Templated_Transform.h"

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
  params.tunable_params.upper_base_z_offset = T(0);
  params.tunable_params.D1 = T(62.5);
  params.tunable_params.D2 = T(62.5);
  params.tunable_params.D3 = T(62.5);
  params.tunable_params.D4 = T(62.5);
  params.lower_base = {T(0), T(224), T(0)};
  params.upper_base = {T(0), T(186), T(0)};
  return params;
}

template <typename T> inline T *Parameters_to_array(Parameters<T> parameters) {
  T *array = new T[35];
  Tunable_parameters<T> tunables = parameters.tunable_params;
  for (int i = 0; i < num_loops; ++i) {
    array[i * 6 + 0] = tunables.loop_parameters[i].x_slider_offset;
    array[i * 6 + 1] = tunables.loop_parameters[i].y_slider_offset;
    array[i * 6 + 2] = tunables.loop_parameters[i].transmission_link_length;
    array[i * 6 + 3] = tunables.loop_parameters[i].proximal_link_length;
    array[i * 6 + 4] = tunables.loop_parameters[i].proximal_link_midpoint;
    array[i * 6 + 5] = tunables.loop_parameters[i].distal_link_length;
  }
  array[24] = tunables.top_needle_to_holder_distance;
  array[25] = tunables.bottom_needle_to_holder_distance;
  array[26] = tunables.top_holder_to_linkage_distance;
  array[27] = tunables.bottom_holder_to_linkage_distance;
  array[28] = tunables.needle_offset;
  array[29] = tunables.lower_base_z_offset;
  array[30] = tunables.upper_base_z_offset;
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
  p.tunable_params.top_needle_to_holder_distance = tunable_params[24];
  p.tunable_params.bottom_needle_to_holder_distance = tunable_params[25];
  p.tunable_params.top_holder_to_linkage_distance = tunable_params[26];
  p.tunable_params.bottom_holder_to_linkage_distance = tunable_params[27];
  p.tunable_params.needle_offset = tunable_params[28];
  p.tunable_params.lower_base_z_offset = tunable_params[29];
  p.tunable_params.upper_base_z_offset = tunable_params[30];
  p.tunable_params.D1 = tunable_params[31];
  p.tunable_params.D2 = tunable_params[32];
  p.tunable_params.D3 = tunable_params[33];
  p.tunable_params.D4 = tunable_params[34];
  return p;
}

template <typename T>
inline Transform<T> get_end_effector(Thetas<T> thetas, Parameters<T> parameters);

template <typename T>
inline Point<T> get_upper_linkage_n_vec(Thetas<T> thetas, Parameters<T> parameters);

template <typename T>
inline Point<T> get_upper_linkage_E(Thetas<T> thetas, Parameters<T> parameters);

template <typename T>
inline Point<T> get_lower_linkage_E(Thetas<T> thetas, Parameters<T> parameters);

template <typename T>
inline Point<T> get_D(Thetas<T> thetas, Parameters<T> parameters);


template <typename T>
inline Point<T> get_upper_linkage_P(Thetas<T> thetas, Parameters<T> parameters);


template <typename T>
inline Point<T> get_lower_linkage_P(Thetas<T> thetas, Parameters<T> parameters);

template <typename T>
inline Point<T> get_linkage_C(Thetas<T> thetas, Parameters<T> parameters,
                       linkage_values val);

template <typename T>
inline Point<T> get_linkage_B(Thetas<T> thetas, Parameters<T> parameters,
                       linkage_values val);

//template <typename T>
//inline Thetas<T> inverse_kinematics(Parameters<T> parameters, Transform<T> transform);

//     l1
//  P3____
// l2\    P2
//    \ t/ l3
//     P1
//
template <typename T>
inline Point<T> third_point_in_triangle(Point<T> P1, Point<T> P2, T l1, T l2) {
  T z = P1.z;
  P1.z = 0;
  P2.z = 0;
  Point<T> P3;
  Point<T> diff = P2 - P1;
  Point<T> norm = diff.normalize();
  // we will always rotate clockwize from diff.
  Point<T> rotated_norm = {-norm.y, norm.x, 0};
  T l3 = (P1 - P2).magnitude();

  T l2cos_t = (l2 * l2 + l3 * l3 - l1 * l1) / (2 * l3);
  T h = std::sqrt(l2 * l2 - l2cos_t * l2cos_t);
  P3 = P1 + (l2cos_t * norm) + (h * rotated_norm);
  P3.z = z;
  return P3;
};

#endif // CAL_KINEMATICS
