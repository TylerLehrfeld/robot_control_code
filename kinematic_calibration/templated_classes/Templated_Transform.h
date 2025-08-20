#ifndef TEMPLATED_TRANSFORM
#define TEMPLATED_TRANSFORM

#include "Templated_Point.h"
#include <cmath>

template <typename T> class Rotation {
public:
  T matrix[3][3] = {};
  Rotation<T>() {
    for (int i = 0; i < 3; ++i) {
      matrix[i][i] = T(1);
    }
  }
  Rotation<T>(T theta_x, T theta_y, T theta_z) {

    Rotation<T> X;
    Rotation<T> Y;
    Rotation<T> Z;

    T cx = cos(theta_x);
    T cy = cos(theta_y);
    T cz = cos(theta_z);
    T sx = sin(theta_x);
    T sy = sin(theta_y);
    T sz = sin(theta_z);
    T X_matrix[3][3] = {{T(1), T(0), T(0)}, {T(0), cx, -sx}, {T(0), sx, cx}};
    T Y_matrix[3][3] = {{cy, T(0), sy}, {T(0), T(1), T(0)}, {-sy, T(0), cy}};
    T Z_matrix[3][3] = {{cz, -sz, T(0)}, {sz, cx, T(0)}, {T(0), T(0), T(1)}};

    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        X.matrix[i][j] = X_matrix[i][j];
        Y.matrix[i][j] = Y_matrix[i][j];
        Z.matrix[i][j] = Z_matrix[i][j];
      }
    }
    Rotation<T> R = Z * Y * X;
    *this = R;
  }

  Rotation<T>(const Rotation<T> &R) {
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        matrix[i][j] = R.matrix[i][j];
      }
    }
  }
  Rotation<T> operator*(const Rotation<T> &R1) const {
    Rotation<T> ret;
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        ret.matrix[i][j] = T(0);
        for (int k = 0; k < 3; k++) {
          ret.matrix[i][j] += matrix[i][k] * R1.matrix[k][j];
        }
      }
    }
    return ret;
  }
  Point<T> operator*(const Point<T> &p) const {
    Point<T> ret;
    ret.x = p.x * matrix[0][0] + p.y * matrix[0][1] + p.z * matrix[0][2];
    ret.y = p.x * matrix[1][0] + p.y * matrix[1][1] + p.z * matrix[1][2];
    ret.z = p.x * matrix[2][0] + p.y * matrix[2][1] + p.z * matrix[2][2];
    return ret;
  }
  Rotation inverse() {
    Rotation<T> inv;
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        inv.matrix[i][j] = matrix[j][i];
      }
    }
    return inv;
  }
  template <typename U> Rotation<U> convert() const {
    Rotation<U> rot;
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        rot.matrix[i][j] = U(rot.matrix[i][j]);
      }
    }
    return rot;
  }
};

template <typename T> class Transform {
public:
  Rotation<T> R;
  Point<T> p;
  Transform<T>() {
    R = Rotation<T>(T(0), T(0), T(0));
    p = {T(0), T(0), T(0)};
  }
  Transform<T>(T theta_x, T theta_y, T theta_z, T x, T y, T z) {
    R = Rotation<T>(theta_x, theta_y, theta_z);
    p = {x, y, z};
  }
  Transform<T>(Point<T> x_hat, Point<T> y_hat, Point<T> z_hat,
               Point<T> translation) {
    R.matrix[0][0] = x_hat.x;
    R.matrix[1][0] = x_hat.y;
    R.matrix[2][0] = x_hat.z;
    R.matrix[0][1] = y_hat.x;
    R.matrix[1][1] = y_hat.y;
    R.matrix[2][1] = y_hat.z;
    R.matrix[0][2] = z_hat.x;
    R.matrix[1][2] = z_hat.y;
    R.matrix[2][2] = z_hat.z;
    p = translation;
  }

  Transform<T> operator*(const Transform<T> &transform) const {
    Transform<T> ret;
    ret.R = R * transform.R;
    ret.p = (R * transform.p) + p;
    return ret;
  };

  Point<T> operator*(const Point<T> &p2) const { return R * p2 + p; }
  Transform<T> inverse() {
    Transform<T> ret;
    ret.R = R.inverse();
    ret.p = ret.R * (T(-1) * p);
    return ret;
  }
  template <typename U> Transform<U> convert() const {
    Transform<U> transform;
    transform.p = p.template convert<U>();
    transform.R = R.template convert<U>();
    return transform;
  }
};

#endif // TEMPLATED_TRANSFORM
