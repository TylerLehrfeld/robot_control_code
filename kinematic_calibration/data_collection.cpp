#include "atracsys_functions.h"
#include "inverse_kinematics.h"
#include "kinematics.h"
#include "robot_controller.h"
#include "templated_classes/Templated_Point.h"
#include "templated_classes/Templated_Transform.h"
#include <cmath>
#include <fstream>
#include <iomanip>
#include <vector>
Transform<double> F_M1R(0,0,0,0, -5.5, -(67 + 43.5));
Transform<double> F_M2N(0, 0, M_PI, .00558085, 28.3789, -5.49509);
Transform<double> F_NEE(const Thetas<double> &thetas,
                        const Parameters<double> &params) {
  Point P1 = get_upper_linkage_P(thetas, params);
  Point P2 = get_lower_linkage_P(thetas, params);
  Point<double> z = (P1 - P2).normalize();
  Point<double> x =
      cross(get_upper_linkage_n_vec(thetas, params), z).normalize();
  Point<double> y = cross(z, x);
  Transform<double> N_Frame(x, y, z, P2);
  Transform<double> EE_frame = get_end_effector(thetas, params);
  return N_Frame.inverse() * EE_frame;
}
Transform<double> F_NEE(const Thetas<double> &thetas) {
  return F_NEE(thetas, get_default_parameters<double>());
};

void open_log_file(std::ofstream &logfile) {
  logfile << "index," << "target_x,target_y,target_z,"
          << "actual_x,actual_y,actual_z,"
          << "targeting_error_mm,angular_error_deg,";

  for (int r = 0; r < 3; r++) {
    for (int c = 0; c < 3; c++) {
      logfile << "exp_R" << r << c << ",";
    }
  }
  for (int r = 0; r < 3; r++) {
    for (int c = 0; c < 3; c++) {
      logfile << "act_R" << r << c;
      if (!(r == 2 && c == 2))
        logfile << ",";
    }
  }
  logfile << "\n";

  logfile << std::fixed << std::setprecision(6);
}

void write_to_log_file(std::ofstream &logfile, Transform<double> expected_F_EE,
                       Transform<double> actual_F_EE, double targeting_error,
                       double angular_error, int index) {
  // write row
  logfile << index << "," << expected_F_EE.p.x << "," << expected_F_EE.p.y
          << "," << expected_F_EE.p.z << "," << actual_F_EE.p.x << ","
          << actual_F_EE.p.y << "," << actual_F_EE.p.z << "," << targeting_error
          << "," << angular_error << ",";

  // expected rotation
  for (int r = 0; r < 3; r++) {
    for (int c = 0; c < 3; c++) {
      logfile << expected_F_EE.R.matrix[r][c] << ",";
    }
  }

  // actual rotation
  for (int r = 0; r < 3; r++) {
    for (int c = 0; c < 3; c++) {
      logfile << actual_F_EE.R.matrix[r][c];
      if (!(r == 2 && c == 2))
        logfile << ",";
    }
  }

  logfile << "\n";
}
int main() {

  std::ofstream logfile("experiment_results.csv");
  open_log_file(logfile);
  // get F_M1R
  // get F_M2EE
  // backlash flag
  bool backlash_compensation = true;
  // targeting positions in vector
  std::vector<Thetas<double>> positions;
  for (int i = 375; i < 445; i += 5) {
    Transform<double> target_transform(0, 0, 0, 0, i, -65);
    positions.push_back(get_thetas(target_transform));
  }
  AtracsysTracker<double> atracsys("geometry100000.ini","geometry999.ini");
  RobotController robot(backlash_compensation);
  //  robot.home();
  // enter for loop:
  int index = 0;
  for (Thetas<double> thetas : positions) {
    thetas.print();
    // go to targetted position based on inverse kinematics
    // get encoder position error to account for it
    encoder_error_struct errs = robot.move(thetas);
    // calculate expected F_EE
    Thetas<double> encoder_adjusted_thetas = thetas;
    encoder_adjusted_thetas.theta_1 += errs.mmErrLeft;
    encoder_adjusted_thetas.theta_2 += errs.mmErrRight;
    encoder_adjusted_thetas.theta_3 += errs.mmErrLeftMiddle;
    encoder_adjusted_thetas.theta_4 += errs.mmErrRightMiddle;
    Transform<double> expected_F_EE = get_end_effector(encoder_adjusted_thetas);
    // get measurement
    Measurement<double> atracsys_measurement;
    while (atracsys.getMeasurement(BOTH, atracsys_measurement) < 0) {
      std::this_thread::sleep_for(750ms);
    };
    // get positional error of measurement
    // get angular error of needle and positional error of needle
    Transform<double> actual_F_EE =
        F_M1R.inverse() * atracsys_measurement.F_OM1.inverse() *
        atracsys_measurement.F_OM2 * F_M2N * F_NEE(thetas);
    double targeting_error = (actual_F_EE.p - expected_F_EE.p).magnitude();
    Point<double> expected_z = {expected_F_EE.R.matrix[0][2],
                                expected_F_EE.R.matrix[1][2],
                                expected_F_EE.R.matrix[2][2]};
    Point<double> actual_z = {actual_F_EE.R.matrix[0][2],
                              actual_F_EE.R.matrix[1][2],
                              actual_F_EE.R.matrix[2][2]};
    double angular_error = 180 / M_PI * std::acos(expected_z * actual_z);

    // write all to file
    write_to_log_file(logfile, expected_F_EE, actual_F_EE, targeting_error,
                      angular_error, index);
    index++;
  } // exit loop
}
