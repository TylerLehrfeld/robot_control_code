# Program Overview
This program includes all of the code used to control the robot, do kinematics and analysis.
# Files
3D_slicer_interface.h: Gets and sends information from and to 3D slicer
control.cpp: Control flow code that makes the robot move
forward_kinematics.cpp: Implements forward kinematic functions
forward_kinematics.h: Defines forward kinematic mathematical functions
galil_control_calls.cpp: Implements functions that send commands to the robot through galil
inverse_kinematics_test.cpp: Tests whether the inverse kinematic code is working correctly
inver.cpp: Implements inverse kinematic mathematical functions
inverse_kinematics.h: Defines inverse kinematic functions
kinematic_structs.h: Defines kinematic structs passed into and returned from functions
Kinematics.pdf: A report on the forward and inverse kinematics of the code and its error implications
kinematics_end_effector_analysis_test.cpp: A test script to get certain slider positions under certain conditions
kinematics_movement_test.cpp: A test script to move the robot to a certain position
kinematics_test.cpp: Test the forward kinematics
Makefile: make all the executables
Point.h: Define the point struct and implement its functions
Transform_test.cpp: Test the Transform class
Transform.h: Define the Transform class 