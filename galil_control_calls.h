
#include "forward_kinematics.h"
#include "gclib.h"
#include "gclibo.h"

#ifndef GALIL_CALLS
#define GALIL_CALLS
const char* GALIL_IP_STRING = "192.168.1.10"; 


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


#endif