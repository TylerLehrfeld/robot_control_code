CXX = g++
CXXFLAGS = -g -Wall -Wextra -std=c++17

# Source files
SRC = inverse_kinematics_test.cpp \
      forward_kinematics.cpp \
      inverse_kinematics.cpp

# Header files
HEADERS = forward_kinematics.h \
          inverse_kinematics.h \
          kinematic_structs.h

# Object files
OBJ = $(SRC:.cpp=.o)

# Output executable
TARGET = inverse_kinematics_test

all: $(TARGET)

$(TARGET): $(OBJ)
	$(CXX) $(CXXFLAGS) -o $@ $^

%.o: %.cpp $(HEADERS)
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	rm -f $(OBJ) $(TARGET)
