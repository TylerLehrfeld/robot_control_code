#ifndef EXPERIMENT_HELPERS
#define EXPERIMENT_HELPERS

#include <fstream>
#include "atracsys_functions.h"
#include "kinematics.h"
#include "templated_classes/Templated_Point.h"
#include "templated_classes/Templated_Transform.h"
#include <cmath>
#include <iomanip>
#include "jacobian.h"

void open_log_file(std::ofstream &logfile) {
  logfile << "index," << "target_x,target_y,target_z,"
          << "actual_x,actual_y,actual_z,"
          << "targeting_error_mm,angular_error_deg,";

  // expected EE rotation
  for (int r = 0; r < 3; ++r)
    for (int c = 0; c < 3; ++c)
      logfile << "exp_R" << r << c << ",";

  // actual EE rotation
  for (int r = 0; r < 3; ++r)
    for (int c = 0; c < 3; ++c)
      logfile << "act_R" << r << c << ",";

  // F_OM1 position
  logfile << "F_OM1_x,F_OM1_y,F_OM1_z,";

  // F_OM1 rotation
  for (int r = 0; r < 3; ++r)
    for (int c = 0; c < 3; ++c)
      logfile << "F_OM1_R" << r << c << ",";

  // F_OM2 position
  logfile << "F_OM2_x,F_OM2_y,F_OM2_z,";

  // F_OM2 rotation
  for (int r = 0; r < 3; ++r)
    for (int c = 0; c < 3; ++c) {
      logfile << "F_OM2_R" << r << c;
      if (!(r == 2 && c == 2))
        logfile << ",";
    }

  logfile << "\n";

  logfile << std::fixed << std::setprecision(6);
}

void write_to_log_file(std::ofstream &logfile, const Transform<double> &F_OM1,
                       const Transform<double> &F_OM2,
                       const Transform<double> &expected_F_EE,
                       const Transform<double> &actual_F_EE,
                       double targeting_error, double angular_error,
                       int index) {
  logfile << std::fixed << std::setprecision(8);

  // Basic info
  logfile << index << "," << expected_F_EE.p.x << "," << expected_F_EE.p.y
          << "," << expected_F_EE.p.z << "," << actual_F_EE.p.x << ","
          << actual_F_EE.p.y << "," << actual_F_EE.p.z << "," << targeting_error
          << "," << angular_error << ",";

  // Expected EE rotation
  for (int r = 0; r < 3; ++r)
    for (int c = 0; c < 3; ++c)
      logfile << expected_F_EE.R.matrix[r][c] << ",";

  // Actual EE rotation
  for (int r = 0; r < 3; ++r)
    for (int c = 0; c < 3; ++c)
      logfile << actual_F_EE.R.matrix[r][c] << ",";

  // F_OM1 position
  logfile << F_OM1.p.x << "," << F_OM1.p.y << "," << F_OM1.p.z << ",";

  // F_OM1 rotation
  for (int r = 0; r < 3; ++r)
    for (int c = 0; c < 3; ++c)
      logfile << F_OM1.R.matrix[r][c] << ",";

  // F_OM2 position
  logfile << F_OM2.p.x << "," << F_OM2.p.y << "," << F_OM2.p.z << ",";

  // F_OM2 rotation
  for (int r = 0; r < 3; ++r)
    for (int c = 0; c < 3; ++c) {
      logfile << F_OM2.R.matrix[r][c];
      if (!(r == 2 && c == 2))
        logfile << ",";
    }

  logfile << "\n";
}

void get_errors(Measurement<double> &atracsys_measurement,
                Thetas<double> &thetas, Parameters<double> &params,
                Transform<double> &expected_F_EE, int index,
                std::ofstream &logfile) {

  Transform<double> actual_F_EE =
      F_M1R<double>().inverse() * atracsys_measurement.F_OM1.inverse() *
      atracsys_measurement.F_OM2 * F_M2N<double>(0, params) *
      F_NEE(thetas, params);
  double targeting_error = (actual_F_EE.p - expected_F_EE.p).magnitude();
  if (targeting_error > 50) {
    atracsys_measurement.F_OM1 =
        atracsys_measurement.F_OM1 * Transform<double>(0, 0, M_PI, 0, 0, 0);
    actual_F_EE = F_M1R<double>().inverse() *
                  atracsys_measurement.F_OM1.inverse() *
                  atracsys_measurement.F_OM2 * F_M2N<double>(0, params) *
                  F_NEE(thetas, params);
    targeting_error = (actual_F_EE.p - expected_F_EE.p).magnitude();
  }
  Point<double> expected_z = {expected_F_EE.R.matrix[0][2],
                              expected_F_EE.R.matrix[1][2],
                              expected_F_EE.R.matrix[2][2]};
  Point<double> actual_z = {actual_F_EE.R.matrix[0][2],
                            actual_F_EE.R.matrix[1][2],
                            actual_F_EE.R.matrix[2][2]};
  double angular_error = 180 / M_PI * std::acos(expected_z * actual_z);

  // write all to file
  write_to_log_file(logfile, atracsys_measurement.F_OM1,
                    atracsys_measurement.F_OM2, expected_F_EE, actual_F_EE,
                    targeting_error, angular_error, index);
}

#endif // !EXPERIMENT_HELPERS
