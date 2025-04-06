# Makefile for ini_to_desmos.cpp

CXX = g++
PATHS = ../PROGRAMS/dependencies/eigen-3.4.0

CXXFLAGS = -std=c++17 -g -I $(PATHS)
TARGETS = parse_ini orient_ini
SRC = ini_to_desmos.cpp \
orient_ini.cpp

all: $(TARGETS)

parse_ini: $(SRC)
	$(CXX) $(CXXFLAGS) -o $(TARGET) $(SRC)

orient_ini: $(SRC)
	$(CXX) $(CXXFLAGS) -o $(TARGET) $(SRC)


clean:
	rm -f $(TARGET)

.PHONY: all clean

# Compiler and flags
CXX = g++
CXXFLAGS = -std=c++17 -O2 -Wall

# Project structure
SRC_DIR = .
PROG_DIR = ./PROGRAMS

# Source files
SRCS = $(SRC_DIR)/orient_ini.cpp \
       $(SRC_DIR)/ini_helpers.cpp \
       $(PROG_DIR)/Matrix.cpp \
       $(PROG_DIR)/helperFunctions.cpp

# Object files
OBJS = $(SRCS:.cpp=.o)

# Output binary
TARGETS = orient_ini 

# Build target
all: $(TARGET)

$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $^

# Compile source to object
%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Clean up
clean:
	rm -f $(TARGET) $(OBJS)

.PHONY: all clean


CXX = g++
CXXFLAGS = -g -Wall -Wextra -std=c++17

# Source files
SRC = 
# Header files
HEADERS = ../PROGRAMS/Matrix.h \


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

%.o: %.cpp $(HEADERS)
	$(CXX) $(CXXFLAGS) -c $< -o $@


clean:
	rm -f $(OBJ) $(TARGETS)

