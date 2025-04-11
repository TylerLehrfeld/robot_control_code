
#include "kinematic_structs.h"
#include "gclib.h"
#include "gclibo.h"
#include <sstream>
#include <string>
#include <chrono>
#include <thread>
using namespace std::chrono_literals;
#ifndef GALIL_CALLS
#define GALIL_CALLS

GCStringIn GALIL_IP_STRING = "192.168.1.10"; 
GCon g = NULL;

void init_galil() {
    char addresses[G_SMALL_BUFFER];
    GReturn rc;
    rc = GAddresses(addresses, G_SMALL_BUFFER);
    printf("%s", addresses);
    
    char buf[G_SMALL_BUFFER];
    
    rc = GOpen(GALIL_IP_STRING, &g);
    std::cout << "return val: " << rc << std::endl;
}


/**
 * @brief use the slider positions to move the robot into a desired position
 * 
 * @param positions 
 */
void move_robot_with_slider_positions(slider_positions positions) {
    char buf[G_SMALL_BUFFER];
    GSize read_bytes = 0;
    double right = BASE_TO_SLIDER_MAX - positions.right_slider_y;
    double left = BASE_TO_SLIDER_MAX - positions.left_slider_y;
    double right_middle = BASE_TO_SLIDER_MAX - positions.right_middle_slider_y;
    double left_middle = BASE_TO_SLIDER_MAX - positions.left_middle_slider_y;
    std::stringstream ss;
    ss << "tgtMmA = " << right;
    GCStringIn command = ss.str().c_str();
    ss.clear();
    GCommand(g, command, buf, G_SMALL_BUFFER, &read_bytes);
    ss << "tgtMmF = " << left;
    command = ss.str().c_str();
    GCommand(g, command, buf, G_SMALL_BUFFER, &read_bytes);
    ss.clear();
    ss << "tgtMmB = " << right_middle;
    command = ss.str().c_str();
    GCommand(g, command, buf, G_SMALL_BUFFER, &read_bytes);
    ss.clear();
    ss << "tgtMmE = " << left_middle;
    command = ss.str().c_str();
    GCommand(g, command, buf, G_SMALL_BUFFER, &read_bytes);
    GCommand(g, "XQ #GoToLow, 1", buf, G_SMALL_BUFFER, &read_bytes);
    GCommand(g, "XQ #GoToUp, 1", buf, G_SMALL_BUFFER, &read_bytes);
    std::this_thread::sleep_for(100ms);
    int value = 0;
    do {
        GCmdI(g, "dMotion = ?", &value);
        std::this_thread::sleep_for(500ms);
    } while(!value);
    std::this_thread::sleep_for(1750ms);
    
    
}


#endif