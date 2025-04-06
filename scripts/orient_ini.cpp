/**
 * @file orient_ini.cpp
 * @author Tyler Lehrfeld (tlehrfe2@jhu.edu)
 * @brief This file orients a certain marker file (.ini)
 *  such that its rotation frame corresponds with the needle frame
 * @version 0.1
 * @date 2025-04-05
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "ini_helpers.h"
#include <cmath>
#include "../PROGRAMS/Matrix.h"
#include "../PROGRAMS/helperFunctions.h"

int main(int argc, char **argv)
{
    std::string filepath = std::string(argv[1]);
    // We want fiducials 2 and 4 to be oriented upwards. Fiducial 2 should be at the origin.
    std::vector<Point> fiducials = parse_ini_file(filepath);
    std::vector<Matrix> Matrix_fiducials;
    // put fiducial 2 at origin
    for (int i = 0; i < fiducials.size(); i++)
    {
        fiducials[i] = fiducials[i] - fiducials[1];
        Matrix_fiducials.push_back(fiducials[i].to_matrix());
    }
    // rotate fiducials such that fiducial 4 has an x and y of zero or (fiducial 4 is above 2).
    Point normalized_4 = fiducials[3].normalize();
    Point rotation_axis_point = cross(normalized_4, {0, 0, 1});
    double theta = acos(normalized_4 * rotation_axis_point);
    Matrix I = generate_identity(3);
    Matrix rotation_axis(3, 1, {rotation_axis_point.x, rotation_axis_point.y, rotation_axis_point.z});
    Matrix n_x(3, 3, 
        {
            0, -rotation_axis_point.z, -rotation_axis_point.y, 
            rotation_axis_point.z, 0, -rotation_axis_point.x, 
            -rotation_axis_point.y, rotation_axis_point.x, 0
        });
    Matrix R = I * cos(theta) + (1 - cos(theta)) * (rotation_axis * rotation_axis.transpose()) + n_x;
    for(int i = 0; i < Matrix_fiducials.size(); i++) {
        Matrix_fiducials[i] = R*Matrix_fiducials[i];
    }

    for (size_t i = 0; i < fiducials.size(); ++i) {
        Point f = fiducials[i];
        std::cout << "(" << f.x << ", " << f.y << ", " << f.z << ")\n";
    }
}