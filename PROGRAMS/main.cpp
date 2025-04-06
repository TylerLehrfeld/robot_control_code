#include <Eigen/Dense>
#include <iostream>

#include "Matrix.h"
#include "Pivot.h"
#include "PointCloudTransform.h"
#include "Transform.h"
#include "dataReaderHelpers.h"
#include "distortionCorrection.h"
#include "mainHelper.h"
/**
 * @brief The main executable that creates the output files
 *
 * @return int
 */
int main() {
    // get data from input files
    int number_of_testcases = get_number_of_test_cases();
    vector<Calbody_data> calbody_readings = read_calbody_data();
    vector<Calreadings_data> calreadings_readings = read_calreadings_data();
    vector<EMPivot_data> empivot_readings = read_empivot_data();
    vector<OPTPivot_data> optpivot_readings = read_optpivot_data();
    vector<CT_Fiducials_data> ct_fid_readings = read_ct_fiducials_data();
    vector<EM_Fiducials_data> em_fid_readings = read_emfiducials_data();
    vector<EM_Nav_data> em_nav_readings = read_emnav_data();

    // define a pointcloud transformation class
    PointCloudTransform T = PointCloudTransform();

    for(int N = 0; N < number_of_testcases; N++) {
        int numFrames = calreadings_readings[N].N_Frames;
        vector<Matrix> expected_C_i;
        vector<Matrix> measured_C_i;
        // assemble two sets of points: the expected C_i and the measured C_i.
        // To do this, we need F_A and F_D so that F_D^-1*F_A*c_i = expected_C_i
        for(int frame = 0; frame < numFrames; frame++) {
            Transform F_A = compute_F_A(frame, T, calreadings_readings[N],
                                        calbody_readings[N]);
            Transform F_D = compute_F_D(frame, T, calreadings_readings[N],
                                        calbody_readings[N]);
            Transform F_Dinv_F_A = F_D.inverse() * F_A;
            for(int i = 0; i < calbody_readings[N].N_C; i++) {
                Matrix c_i(3, 1,
                           {calbody_readings[N].c_i_coordinates[i][0],
                            calbody_readings[N].c_i_coordinates[i][1],
                            calbody_readings[N].c_i_coordinates[i][2]});
                expected_C_i.push_back(F_Dinv_F_A * c_i);
                double* c_i_vec = calreadings_readings[N].frames[frame][2][i];
                measured_C_i.push_back(
                    Matrix(3, 1, {c_i_vec[0], c_i_vec[1], c_i_vec[2]}));
            }
        }

        // use the expected and measured c_is to compute a distortion correction
        // function
        distortionCorrection distortionClass(expected_C_i, measured_C_i);
        // apply the distortion function to the pivot calculation
        Matrix G_0 = get_G_0(empivot_readings[N], distortionClass);
        vector<Matrix> G_Hat =
            get_G_Hat(empivot_readings[N], distortionClass, G_0);
        vector<Transform> F_Gs =
            get_F_Gs(empivot_readings[N], distortionClass, G_Hat, T);
        Pivot p1 = Pivot(F_Gs);
        Matrix em_pivot_tip = p1.p_t;
        // compute b_j with respect to EM tracking system
        int N_G = em_fid_readings[N].N_G;
        vector<Matrix> b_j_coordinates;
        for(int i = 0; i < em_fid_readings[N].N_B; i++) {
            Transform F_ptr = T.compute(
                G_Hat,
                distortionClass.undistort(em_fid_readings[N].frames[i], N_G));
            b_j_coordinates.push_back(F_ptr * em_pivot_tip);
        }

        // compute F_reg by doing a point cloud registration between computed
        // b_j(in em tracker space) and known b_i (in CT space)
        vector<Matrix> b_i_coordinates;
        for(int i = 0; i < ct_fid_readings[N].N_B; i++) {
            double* b_i_coords = ct_fid_readings[N].b_i_coordinates[i];
            b_i_coordinates.push_back(
                Matrix(3, 1, {b_i_coords[0], b_i_coords[1], b_i_coords[2]}));
        }
        Transform F_reg = T.compute(b_j_coordinates, b_i_coordinates);
        // compute F_reg*ptr_tip for each frame in EM-NAV
        vector<Matrix> v_i_coordinates;
        N_G = em_nav_readings[N].N_G;
        for(int i = 0; i < em_nav_readings[N].N_Frames; i++) {
            Transform F_ptr = T.compute(
                G_Hat,
                distortionClass.undistort(em_nav_readings[N].frames[i], N_G));
            v_i_coordinates.push_back(F_reg * F_ptr * em_pivot_tip);
        }

        create_output_file(v_i_coordinates, em_nav_readings[N].NAME, N);
        if(N < 6) {
            output_average_error(em_nav_readings[N].NAME);
        }
        clear_readings(calbody_readings[N], calreadings_readings[N],
                       empivot_readings[N], optpivot_readings[N],
                       ct_fid_readings[N], em_fid_readings[N],
                       em_nav_readings[N]);
    }
}