CXX = g++
CXXFLAGS = -g -Wall -Wextra -std=c++17

# Source files
SRC = inverse_kinematics_test.cpp \
      kinematics_end_effector_analysis_test.cpp \
      forward_kinematics.cpp \
      inverse_kinematics.cpp \
      forward_inverse_kinematic_comparison.cpp

# Header files
HEADERS = forward_kinematics.h \
          inverse_kinematics.h \
          kinematic_structs.h \
          Robot.h \
          Point.h

# Object files
OBJ = $(SRC:.cpp=.o)

# Output executables
TARGETS = forward_inverse_kinematic_comparison #inverse_kinematics_test kinematics_end_effector_analysis_test

all: $(TARGETS)


#inverse_kinematics_test: inverse_kinematics_test.o forward_kinematics.o inverse_kinematics.o
#	$(CXX) $(CXXFLAGS) -o $@ $^

#kinematics_end_effector_analysis_test: kinematics_end_effector_analysis_test.o forward_kinematics.o inverse_kinematics.o
#	$(CXX) $(CXXFLAGS) -o $@ $^
#
forward_inverse_kinematic_comparison: forward_inverse_kinematic_comparison.o forward_kinematics.o inverse_kinematics.o
	$(CXX) $(CXXFLAGS) -o $@ $^

%.o: %.cpp $(HEADERS)
	$(CXX) $(CXXFLAGS) -c $< -o $@


clean:
	rm -f $(OBJ) $(TARGETS)
