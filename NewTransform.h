/**
 * @file NewTransform.h
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
#include "Transform.h"

#ifndef NEW_TRANSFORM
#define NEW_TRANSFORM


class NewTransform {
public:
  double matrix[4][4];
  NewTransform() { base_constructor(); }

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
  NewTransform(double theta_x, double theta_y, double theta_z, double x, double y,
            double z) {
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

  NewTransform(Point translation, double rotation[3][3]) {
    matrix[0][0] = rotation[0][0];
    matrix[0][1] = rotation[0][1];
    matrix[0][2] = rotation[0][2];
    matrix[0][3] = translation.x;
    matrix[1][0] = rotation[1][0];
    matrix[1][1] = rotation[1][1];
    matrix[1][2] = rotation[1][2];
    matrix[1][3] = translation.x;
    matrix[2][0] = rotation[2][0];
    matrix[2][1] = rotation[2][1];
    matrix[2][2] = rotation[2][2];
    matrix[2][3] = translation.x;
    matrix[3][0] = 0;
    matrix[3][1] = 0;
    matrix[3][2] = 0;
    matrix[3][3] = 1;
    
    
  }

  /**
   * @brief Destroy the Transform object
   * 
   */
  ~NewTransform() {
  }

  
  /**
   * @brief multiply transforms together
   * 
   * @param T1 
   * @return Transform 
   */
  NewTransform operator*(NewTransform &T1) {
    NewTransform new_transform;
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


  bool operator==(NewTransform &T) {
    for(int i = 0; i < 4; i++) {
      for(int j = 0; j < 4; j++) {
        if(this->matrix[i][j] != T.matrix[i][j]) {
          return false;
        }
      }
    }
    return true;
  }
  /**
   * @brief return the inverse of the transform matrix
   * 
   * @return NewTransform 
   */
  NewTransform inverse() {
      NewTransform inv;
      double rot[3][3];
      double trans[3];
      
      // Extract rotation matrix
      for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
          rot[i][j] = matrix[i][j];
        }
        trans[i] = matrix[i][3]; // Extract translation vector
      }
      
      // Compute inverse rotation (transpose of rotation matrix)
      for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
          inv.matrix[i][j] = rot[j][i];
        }
      }
      
      // Compute inverse translation
      for (int i = 0; i < 3; i++) {
        inv.matrix[i][3] = -(inv.matrix[i][0] * trans[0] + inv.matrix[i][1] * trans[1] + inv.matrix[i][2] * trans[2]);
      }
      
      // Set last row
      inv.matrix[3][0] = 0;
      inv.matrix[3][1] = 0;
      inv.matrix[3][2] = 0;
      inv.matrix[3][3] = 1;
      
      return inv;
    
  }

  Transform to_transform() {
    Transform T;
    T.p_AB = Matrix(3,1,{matrix[0][3], matrix[1][3], matrix[2][3]});
    T.R_AB = Matrix(3,3, {
      matrix[0][0], matrix[0][1], matrix[0][2],
      matrix[1][0], matrix[1][1], matrix[1][2], 
      matrix[2][0], matrix[2][1], matrix[2][2]});
    return T;
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