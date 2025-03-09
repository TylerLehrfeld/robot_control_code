/**
 * @file 3D_slicer_interface.h
 * @author Tyler Lehrfeld
 * @brief This file is the file that communicates with 3D slicer to move the stl files according to their kinematically determined transforms.
 * @version 0.1
 * @date 2025-03-08
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#ifndef SLICER_INTERFACE
#define SLICER_INTERFACE

#include "kinematic_structs.h"

approach_definition get_approach_from_3D_slicer_UI();


#endif