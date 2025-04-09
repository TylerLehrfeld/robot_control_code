#include <cmath>

#include <iostream>
#include <vector>

#include "Matrix.h"
#include "helperFunctions.h"

#ifndef DISTORT
#define DISTORT

class distortionCorrection {
private:
    Matrix c_matrix;
    Matrix F_matrix;
    Matrix P_matrix;
    Matrix x_min;
    Matrix x_max;

    /**
     * @brief interpolation polynomial using 5th degree bernstien polynomials
     *
     * @param i
     * @param j
     * @param k
     * @param u
     * @return double
     */
    double F(int i, int j, int k, Matrix u) {
        return B(5, i, u.matrixArray[0]) * B(5, j, u.matrixArray[1]) *
               B(5, k, u.matrixArray[2]);
    }

    /**
     * @brief bernstien polynomials
     *
     * @param N
     * @param k
     * @param v
     * @return double
     */
    double B(int N, int k, double v) {
        return (double)(choose(N, k)) * pow(1 - v, N - k) * pow(v, k);
    }

    /**
     * @brief define the scale box vectors x_min and x_max using the unknown
     * coordinates q
     *
     * @param q
     */
    void defineScaleToBox(vector<Matrix> q) {
        x_min = q[0];
        x_max = q[0];
        for(int i = 1; i < q.size(); i++) {
            for(int j = 0; j < 3; j++) {
                if(q[i].matrixArray[j] < x_min.matrixArray[j]) {
                    x_min.matrixArray[j] = q[i].matrixArray[j];
                }
                if(q[i].matrixArray[j] > x_max.matrixArray[j]) {
                    x_max.matrixArray[j] = q[i].matrixArray[j];
                }
            }
        }
    }

    /**
     * @brief scales a v vector so that is it smaller and positive so the
     * bernstien polynomials are more accurate
     *
     * @param v
     * @return Matrix
     */
    Matrix scaleToBox(Matrix v) {
        for(int i = 0; i < 3; i++) {
            v.matrixArray[i] = (v.matrixArray[i] - x_min.matrixArray[i]) /
                               (x_max.matrixArray[i] - x_min.matrixArray[i]);
        }
        return v;
    }

public:
    /**
     * @brief Construct a new distortion Correction object
     *
     * @param p
     * @param q
     */
    distortionCorrection(vector<Matrix> p, vector<Matrix> q) {
        defineScaleToBox(q);
        int num_points = p.size();
        vector<double> matrixArray;
        // define the F_matrix
        F_matrix = Matrix(num_points, 216, matrixArray);
        for(int point_ind = 0; point_ind < num_points; point_ind++) {
            Matrix u = scaleToBox(q[point_ind]);
            for(int i = 0; i <= 5; i++) {
                for(int j = 0; j <= 5; j++) {
                    for(int k = 0; k <= 5; k++) {
                        F_matrix.matrixArray.push_back(F(i, j, k, u));
                    }
                }
            }
        }
        // define p matrix
        vector<double> p_matrix_array;
        for(int point_ind = 0; point_ind < num_points; point_ind++) {
            p_matrix_array.push_back(p[point_ind].matrixArray[0]);
            p_matrix_array.push_back(p[point_ind].matrixArray[1]);
            p_matrix_array.push_back(p[point_ind].matrixArray[2]);
        }
        P_matrix = Matrix(num_points, 3, p_matrix_array);
        // define c matrix
        vector<Matrix> c_columns;
        for(int i = 0; i < 3; i++) {
            vector<double> matrixArr = P_matrix.transpose().matrixArray;
            vector<double> p_columnlist(
                matrixArr.begin() + i * num_points,
                matrixArr.begin() + (i + 1) * num_points);
            Matrix p_column(num_points, 1, p_columnlist);
            Matrix c_column = (F_matrix.transpose() * F_matrix).inverse() *
                              F_matrix.transpose() * p_column;
            c_columns.push_back(c_column);
        }
        c_matrix = Matrix(c_columns);
    }
    ~distortionCorrection() {
    }

    /**
     * @brief undistorts a vector q based on the calculated registration
     *
     * @param q
     * @return Matrix
     */
    Matrix undistort(Matrix q) {
        // this line is used to test how well the program did without correction
        // return q;
        Matrix u = scaleToBox(q);
        Matrix p(3, 1, {0, 0, 0});
        int c = 0;
        for(int i = 0; i <= 5; i++) {
            for(int j = 0; j <= 5; j++) {
                for(int k = 0; k <= 5; k++) {
                    p = p + (Matrix(3, 1,
                                    {c_matrix.matrixArray[3 * c],
                                     c_matrix.matrixArray[3 * c + 1],
                                     c_matrix.matrixArray[3 * c + 2]}) *
                             F(i, j, k, u));

                    c++;
                }
            }
        }
        return p;
    }
    /**
     * @brief undistorts a frame of em_data
     *
     * @param frame
     * @param num_points
     * @return vector<Matrix>
     */
    vector<Matrix> undistort(double** frame, int num_points) {
        vector<Matrix> undistorted_pcloud;
        for(int point_ind = 0; point_ind < num_points; point_ind++) {
            Matrix point(3, 1,
                         {frame[point_ind][0], frame[point_ind][1],
                          frame[point_ind][2]});
            undistorted_pcloud.push_back(undistort(point));
        }
        return undistorted_pcloud;
    }
};

#endif