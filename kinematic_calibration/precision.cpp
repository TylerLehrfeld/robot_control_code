#include <fstream>
#include "helpers.h"
#include "kinematics.h"
#include "inverse_kinematics.h"
#include "atracsys_functions.h"
#include "robot_controller.h"

const bool backlash_compensation = true;

int main() {

  Parameters<double> params = get_default_parameters<double>();
  std::ofstream logfile("precision2.csv");
  open_log_file(logfile);
  AtracsysTracker<double> atracsys("geometry100000.ini", "geometry999.ini");
  Measurement<double> atracsys_measurement;
  RobotController robot(backlash_compensation);

  std::vector<Thetas<double>> positions;
  Transform<double> target_transform;
  Transform<double> home_transform;
  Transform<double> expected_F_EE;
  home_transform = Transform<double>(0, 0, 0, 0, 400, -115);
  for (int i = 0; i < 10; i++) {
    robot.home();
    Thetas<double> thetas = home_positions;
    while (atracsys.getMeasurement(BOTH, atracsys_measurement) < 0) {
      std::this_thread::sleep_for(750ms);
    };
    expected_F_EE = get_end_effector(thetas);
    get_errors(atracsys_measurement, thetas, params, expected_F_EE, i * 2,
               logfile);
    thetas = get_thetas(home_transform);
    robot.move(thetas);
    expected_F_EE = get_end_effector(thetas);
    while (atracsys.getMeasurement(BOTH, atracsys_measurement) < 0) {
      std::this_thread::sleep_for(750ms);
    };
    get_errors(atracsys_measurement, thetas, params, expected_F_EE, i * 2 + 1,
               logfile);
    // thetas = get_thetas(target_transform);
    // robot.move(thetas);
    // expected_F_EE = get_end_effector(thetas);
    // while (atracsys.getMeasurement(BOTH, atracsys_measurement) < 0) {
    //   std::this_thread::sleep_for(750ms);
    // };
    // get_errors(atracsys_measurement, thetas, params, expected_F_EE, i * 2 +
    // 1,
    //            logfile);
  }
}
