# Makefile for ini_to_desmos.cpp

PATHS = ../PROGRAMS/dependencies/eigen-3.4.0 

CXXFLAGS = -std=c++17 -g -I $(PATHS) -I ../PROGRAMS

# Project structure
SRC_DIR = .
PROG_DIR = ../PROGRAMS

# Source files
SRCS = $(PROG_DIR)/Matrix.cpp \
	   $(PROG_DIR)/Transform.cpp \
       


# Header files
HEADERS = ../Point.h \
          ../NewTransform.h \
		  $(PROG_DIR)/Matrix.h \
		  $(PROG_DIR)/Transform.h \
		  $(SRC_DIR)/ini_helpers.h \
		  $(PROG_DIR)/helperFunctions.h

# Object files
OBJS = $(SRCS:.cpp=.o)

# Output binary
TARGETS = orient_ini ini_to_desmos

# Build target
all: $(TARGETS)

orient_ini: orient_ini.o $(OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $^


ini_to_desmos: ini_to_desmos.o $(OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $^

# Compile source to object
%.o: %.cpp $(HEADERS)
	$(CXX) $(CXXFLAGS) -c $< -o $@

	

# Clean up
clean:
	rm -f $(TARGETS) $(OBJS) *.o

.PHONY: all clean
