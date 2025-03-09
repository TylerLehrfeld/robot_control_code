cmake_minimum_required(VERSION 3.10.0)
project(robot_control_code VERSION 0.1.0 LANGUAGES C CXX)

add_executable(robot_control_code 3D_Slicer_Transforms.cpp 3D_slicer_interface.cpp Transform_test.cpp control.cpp forward_kinematics.cpp galil_control_calls.cpp inverse_kinematics.cpp inverse_kinematics_test.cpp kinematics_end_effector_analysis_test.cpp kinematics_movement_test.cpp kinematics_test.cpp)

