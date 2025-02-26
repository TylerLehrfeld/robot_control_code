#ifndef POINT
#define POINT

#include <cmath>
#include <iostream>
struct Point {
  float x;
  float y;
  float z;
  void print() { std::cout << x << " " << y << " " << z << std::endl; }

  /**
   * @brief add a point to another point
   * 
   * @param p2 
   * @return Point 
   */
  Point operator+(Point &p2) {
    return {.x = this->x + p2.x, .y = this->y + p2.y, .z = this->z + p2.z};
  }

  /**
   * @brief subtract p2 from p1 (this)
   * 
   * @param p2 
   * @return Point 
   */
  Point operator-(Point &p2) {
    return {.x = this->x - p2.x, .y = this->y - p2.y, .z = this->z - p2.z};
  }

  /**
   * @brief scale a point
   * 
   * @param scalar 
   * @param m 
   * @return Point 
   */
  friend Point operator*(float scalar, Point &m) {
    return {.x = m.x * scalar, .y = m.y * scalar, .z = m.z * scalar};
  }

  /**
   * @brief scale a point
   * 
   * @param scalar 
   * @return Point 
   */
  Point operator*(float scalar) {
    return {
        .x = this->x * scalar, .y = this->y * scalar, .z = this->z * scalar};
  }

  /**
   * @brief get the magnitude of a vector
   * 
   * @return float 
   */
  float magnitude() {
    return std::sqrt(std::pow(x, 2) + std::pow(y, 2) + std::pow(z, 2));
  }

  /**
   * @brief normalize a vector
   * 
   * @return Point 
   */
  Point normalize() {
    float magnitude = this->magnitude();
    return {.x = this->x / magnitude,
            .y = this->y / magnitude,
            .z = this->z / magnitude};
  }

};

/**
 * @brief return the cross product of two vectors
 * 
 * @param p1 
 * @param p2 
 * @return Point 
 */
inline Point cross(Point p1, Point p2) {
    return {.x = p1.y*p2.z-p1.z*p2.y,.y=p1.z*p2.x-p1.x*p2.z,.z=p1.x*p2.y-p1.y*p2.x};
}

#endif