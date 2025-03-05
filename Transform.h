/**
 * @file Transform.h
 * @author Tyler Lehrfeld
 * @brief This file defines a transformation matrix class that can multiply a 4x4 transform matrix by another to get a new transform
 * @version 0.1
 * @date 2025-02-17
 * 
 * @copyright Copyright (c) 2025
 * 
 */


#include "Point.h"
#include <math.h>
#include <iostream>

#ifndef TRANSFORM
#define TRANSFORM


class Transform {
public:
  float matrix[4][4];
  Transform() { base_constructor(); }

  /**
   * @brief Construct a new Transform object by giving an x angle, a y angle,
   * and a z angle. This will rotate the identity matrix by theta_x, theta_y,
   * theta_z and translate by (x, y, z)
   *
   * @param theta_x rotate around the x axis
   * @param theta_y rotate around the y axis
   * @param theta_z rotate around the z axis
   * @param x
   * @param y
   * @param z
   */
  Transform(float theta_x, float theta_y, float theta_z, float x, float y,
            float z) {
    base_constructor();
    matrix[0][0] = cos(theta_z) * cos(theta_y);
    matrix[0][1] =
        cos(theta_z) * sin(theta_y) * sin(theta_x) - sin(theta_z) * cos(x);
    matrix[0][2] = cos(theta_z) * sin(theta_y) * cos(theta_x) +
                   sin(theta_z) * sin(theta_x);
    matrix[0][3] = x;
    matrix[1][0] = sin(theta_z) * cos(theta_y);
    matrix[1][1] = sin(theta_z) * sin(theta_y) * sin(theta_x) +
                   cos(theta_z) * cos(theta_x);
    matrix[1][2] = sin(theta_z) * sin(theta_y) * cos(theta_x) -
                   cos(theta_z) * sin(theta_x);
    matrix[1][3] = y;
    matrix[2][0] = -sin(theta_y);
    matrix[2][1] = cos(theta_y) * sin(theta_x);
    matrix[2][2] = cos(theta_y) * cos(theta_x);
    matrix[2][3] = z;
    matrix[3][0] = 0;
    matrix[3][1] = 0;
    matrix[3][2] = 0;
    matrix[3][3] = 1;
  }

  /**
   * @brief Destroy the Transform object
   * 
   */
  ~Transform() {
  }

  
  /**
   * @brief multiply transforms together
   * 
   * @param T1 
   * @return Transform 
   */
  Transform operator*(Transform &T1) {
    Transform new_transform;
    for (int i = 0; i < 4; i++) {
      for (int j = 0; j < 4; j++) {
        new_transform.matrix[i][j] = this->matrix[i][0] * T1.matrix[0][j] +
                                     this->matrix[i][1] * T1.matrix[1][j] +
                                     this->matrix[i][2] * T1.matrix[2][j] +
                                     this->matrix[i][3] * T1.matrix[3][j];
      }
    }
    return new_transform;
  }


  /**
   * @brief We transform a point p1 by rotating and translating it.
   * 
   * @param p1 
   * @return Point 
   */
  Point operator*(Point &p1) {
    return {.x = this->matrix[0][0] * p1.x + this->matrix[0][1] * p1.y +
                 this->matrix[0][2] * p1.z + matrix[0][3],
            .y = this->matrix[1][0] * p1.x + this->matrix[1][1] * p1.y +
                 this->matrix[1][2] * p1.z + matrix[1][3],
            .z = this->matrix[2][0] * p1.x + this->matrix[2][1] * p1.y +
                 this->matrix[2][2] * p1.z + matrix[2][3]};
  }

  


  /**
   * @brief The base constructor that creates a new 
   * 
   */
  void base_constructor() {
    
  }

  /**
   * @brief print a transform so that it is readable
   * 
   */
  void print() {
    for(int i = 0; i < 4; i++) {
      for(int j = 0; j < 4; j++) {
        std::cout << matrix[i][j] << " ";
      
      }
      std::cout << std::endl;
    }
  }
};

#endif