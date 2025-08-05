cmake_minimum_required(VERSION 3.18)
project(kinematics_project LANGUAGES CXX)

# Set C++ standard and flags
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Include Eigen
include_directories(${CMAKE_SOURCE_DIR}/dependencies/eigen-3.4.0)
include_directories(${CMAKE_SOURCE_DIR}/dependencies)
include_directories(${CMAKE_SOURCE_DIR}/dependencies/eigen-3.4.0/Eigen)
# Add optional linking libraries
set(POST_LIBS gclib pthread gclibo)

# Common source files reused across targets
set(SRC_SHARED
    forward_kinematics.cpp
    inverse_kinematics.cpp
    Matrix.cpp
    Transform.cpp
)

# If Pivot.cpp is needed
set(SRC_PIVOT
    Pivot.cpp
)

# Targets

add_executable(inverse_kinematics_test
    inverse_kinematics_test.cpp
    ${SRC_SHARED}
)

add_executable(kinematics_end_effector_analysis_test
    kinematics_end_effector_analysis_test.cpp
    ${SRC_SHARED}
)

add_executable(get_test_grid
    get_test_grid.cpp
    ${SRC_SHARED}
)

add_executable(forward_inverse_kinematic_comparison
    forward_inverse_kinematic_comparison.cpp
    ${SRC_SHARED}
)

add_executable(parse_results
    parse_results.cpp
    Matrix.cpp
    Transform.cpp
)

add_executable(Parse_home
    Parse_home.cpp
    Matrix.cpp
    Transform.cpp
)

add_executable(kinematic_grid_error_analysis
    kinematic_grid_error_analysis.cpp
    ${SRC_SHARED}
)

add_executable(galil_control_test
    galil_control_test.cpp
    ${SRC_SHARED}
)
target_link_libraries(galil_control_test PRIVATE ${POST_LIBS})

add_executable(Relative_test
    Relative_test.cpp
    ${SRC_SHARED}
    ${SRC_PIVOT}
)
target_link_libraries(Relative_test PRIVATE ${POST_LIBS})

add_executable(Benchtop_Accuracy_test
    Benchtop_Accuracy_test.cpp
    ${SRC_SHARED}
    ${SRC_PIVOT}
)
target_link_libraries(Benchtop_Accuracy_test PRIVATE ${POST_LIBS})

add_executable(parse_angle
    parse_angle.cpp
    Matrix.cpp
    Transform.cpp
)

add_executable(Home_test
    Home_test.cpp
    ${SRC_SHARED}
    ${SRC_PIVOT}
)
target_link_libraries(Home_test PRIVATE ${POST_LIBS})

