#ifndef SLICER_INTERFACE
#define SLICER_INTERFACE

#include "Point.h"


/**
 * @brief A struct that defines how the robot will approach a target
 * 
 */
struct approach_definition {
    //in mm
    Point target;
    /**
     * Imagine looking at the workspace from behind the robot such that the motors are close and the needle is far.
     * z is up, x is right, and y is forward.
     * Using spherical angle coordinates
     * theta is the angle of the approach away from the z axis: the polar angle θ between the radial line (needle) and a given polar axis (z).
     * phi is the angle to rotate on the x-y plane:  the angle of rotation of the radial line (needle) around the polar axis (z).
     * angles are in degrees
     */
    float theta;
    float phi;
};

/**
 * @brief A struct that defines how the robot will approach a target
 * 
 */
struct target_and_injection_point_approach {
    Point target;
    Point injection_point;
};

inline approach_definition get_approach_from_3D_slicer_UI() {
    //TODO
    return {.target = {.x=0,.y=0,.z=0}, .theta = 0, .phi = 0};
};


#endif