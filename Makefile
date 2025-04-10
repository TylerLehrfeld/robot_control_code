CXX = g++
CXXFLAGS = -g -Wall -Wextra 

# Source files
SRC = inverse_kinematics_test.cpp \
      kinematics_end_effector_analysis_test.cpp \
      Benchtop_Accuracy_test.cpp \
      forward_kinematics.cpp \
      inverse_kinematics.cpp \
      forward_inverse_kinematic_comparison.cpp \
      get_test_grid.cpp

# Header files
HEADERS = forward_kinematics.h \
          inverse_kinematics.h \
          kinematic_structs.h \
          Robot.h \
          Point.h \
          NewTransform.h \
          ../scripts/pivot_needle.h \
          ../scripts/pivot_robot.h

# Object files
OBJ = $(SRC:.cpp=.o)

# Output executables
TARGETS = forward_inverse_kinematic_comparison get_test_grid #inverse_kinematics_test kinematics_end_effector_analysis_test

all: $(TARGETS)


#inverse_kinematics_test: inverse_kinematics_test.o forward_kinematics.o inverse_kinematics.o
#	$(CXX) $(CXXFLAGS) -o $@ $^

#kinematics_end_effector_analysis_test: kinematics_end_effector_analysis_test.o forward_kinematics.o inverse_kinematics.o
#	$(CXX) $(CXXFLAGS) -o $@ $^
#
get_test_grid: get_test_grid.o forward_kinematics.o inverse_kinematics.o
	$(CXX) $(CXXFLAGS) -o $@ $^



forward_inverse_kinematic_comparison: forward_inverse_kinematic_comparison.o forward_kinematics.o inverse_kinematics.o
	$(CXX) $(CXXFLAGS) -o $@ $^



clean:
	rm -f $(OBJ) $(TARGETS)
