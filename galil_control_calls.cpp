const char* GALIL_IP_STRING = "192.168.1.10"; 

#include "3D_slicer_interface.h"
#include "kinematics.h"
#include "gclib.h"
#include "gclibo.h"

/**
 * @brief use the slider positions to move the robot into a desired position
 * 
 * @param positions 
 */
void move_robot_with_slider_positions(slider_positions positions) {
    char addresses[G_SMALL_BUFFER];
    GReturn rc;
    rc = GAddresses(addresses, G_SMALL_BUFFER);
    printf("%s", addresses);
    GCon g = NULL;
    char buf[G_SMALL_BUFFER];
 
    GOpen(GALIL_IP_STRING, &g);
}


/**
 * @brief Get the slider positions with inverse kinematics
 * 
 * @param desired_approach 
 * @return slider_positions 
 */
slider_positions get_slider_positions_with_inverse_kinematics(approach_definition desired_approach) {

}

/**
 * @brief Get the slider positions with inverse kinematics
 * 
 * @param desired_approach 
 * @return slider_positions 
 */
slider_positions get_slider_positions_with_inverse_kinematics(target_and_injection_point_approach desired_approach) {
    
}

/**
 * @brief use approach definitions and inverse kinematics to move the robot as close as possible to a position
 * 
 * @param desired_approach 
 */
 void move_robot_with_inverse_kinematics(approach_definition desired_approach) {
    slider_positions positions = get_slider_positions_with_inverse_kinematics(desired_approach);
    move_robot_with_slider_positions(positions);
}


/**
 * @brief use approach definitions and inverse kinematics to move the robot as close as possible to a position
 * 
 * @param desired_approach 
 */
 void move_robot_with_inverse_kinematics(target_and_injection_point_approach desired_approach) {
    slider_positions positions = get_slider_positions_with_inverse_kinematics(desired_approach);
    move_robot_with_slider_positions(positions);
}
