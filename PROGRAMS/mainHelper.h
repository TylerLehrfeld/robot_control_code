#include <vector>
#include "Transform.h"
#include "PointCloudTransform.h"
#include "dataReaderHelpers.h"
#include "distortionCorrection.h"

/**
 * @brief returns F_A for a specific frame.
 *
 * @param frame frame number to compute F_A
 * @param P_cloud_register a class that runs the point cloud registration
 * @param Calibration_readings a data file object containing the locations
 * of the LEDS on the calibration object
 * @param Calibration_and_em_tracker_data a data file object containing the
 * readings of the optical trackers of the calibration object
 * @return Transform: F_A
 */
Transform compute_F_A(int frame, PointCloudTransform P_cloud_register,
                      Calreadings_data Calibration_readings,
                      Calbody_data Calibration_and_em_tracker_data) {
    double **A_coordinates = Calibration_readings.frames[frame][1];
    double **a_coordinates = Calibration_and_em_tracker_data.a_i_coordinates;
    vector<Matrix> A_cloud;
    vector<Matrix> a_cloud;
    for(int i = 0; i < Calibration_readings.N_A; i++) {
        A_cloud.push_back(Matrix(
            3, 1,
            {A_coordinates[i][0], A_coordinates[i][1], A_coordinates[i][2]}));
        a_cloud.push_back(Matrix(
            3, 1,
            {a_coordinates[i][0], a_coordinates[i][1], a_coordinates[i][2]}));
    }
    return P_cloud_register.compute(a_cloud, A_cloud);
}

/**
 * @brief returns F_D for a specific frame.
 *
 * @param frame frame number to compute F_D
 * @param P_cloud_register a class that runs the point cloud registration
 * @param Calibration_readings a data file object containing the readings of the
 * optical tracker
 * @param Calibration_and_em_tracker_data a data file object containing the
 * locations of optical trackers on the em tracker base
 * @return Transform: F_D
 */
Transform compute_F_D(int frame, PointCloudTransform P_cloud_register,
                      Calreadings_data Calibration_readings,
                      Calbody_data Calibration_and_em_tracker_data) {
    double **D_coordinates = Calibration_readings.frames[frame][0];
    double **d_coordinates = Calibration_and_em_tracker_data.d_i_coordinates;
    vector<Matrix> D_cloud;
    vector<Matrix> d_cloud;
    for(int i = 0; i < Calibration_readings.N_D; i++) {
        D_cloud.push_back(Matrix(
            3, 1,
            {D_coordinates[i][0], D_coordinates[i][1], D_coordinates[i][2]}));
        d_cloud.push_back(Matrix(
            3, 1,
            {d_coordinates[i][0], d_coordinates[i][1], d_coordinates[i][2]}));
    }
    return P_cloud_register.compute(d_cloud, D_cloud);
}

Matrix get_G_0(EMPivot_data empivot_readings_N,
               distortionCorrection distortionClass) {
    Matrix G_0(3, 1, {0, 0, 0});
    int num_em_markers_on_empivot = empivot_readings_N.N_G;
    for(int i = 0; i < num_em_markers_on_empivot; i++) {
        double* G_i_coord = empivot_readings_N.frames[0][i];
        Matrix G_i(3, 1, {G_i_coord[0], G_i_coord[1], G_i_coord[2]});
        G_i = distortionClass.undistort(G_i);
        G_0 = G_0 + G_i;
    }
    G_0 = 1 / (double)(num_em_markers_on_empivot)*G_0;
    return G_0;
}

vector<Matrix> get_G_Hat(EMPivot_data empivot_readings_N, distortionCorrection distortionClass, Matrix G_0) {
    vector<Matrix> G_Hat;
    int num_em_markers_on_empivot = empivot_readings_N.N_G;
    for(int i = 0; i < num_em_markers_on_empivot; i++) {
        double* G_i_coord = empivot_readings_N.frames[0][i];
        Matrix G_i(3, 1, {G_i_coord[0], G_i_coord[1], G_i_coord[2]});
        G_i = distortionClass.undistort(G_i);
        G_Hat.push_back(G_i + -1 * G_0);
    }
    return G_Hat;
}

vector<Transform> get_F_Gs(EMPivot_data empivot_readings_N, distortionCorrection distortionClass, vector<Matrix> G_Hat, PointCloudTransform T) {
    vector<Transform> F_Gs;
    int N_G = empivot_readings_N.N_G;
    for(int i = 0; i < empivot_readings_N.N_Frames; i++) {
        F_Gs.push_back(T.compute(
            G_Hat,
            distortionClass.undistort(empivot_readings_N.frames[i], N_G)));
    }
    return F_Gs;
}
