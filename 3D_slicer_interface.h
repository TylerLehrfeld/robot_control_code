#ifndef SLICER_INTERFACE
#define SLICER_INTERFACE

#include "kinematics.h"




inline approach_definition get_approach_from_3D_slicer_UI() {
    //TODO
    return {.target = {.x=0,.y=0,.z=0}, .theta = 0, .phi = 0};
};


#endif