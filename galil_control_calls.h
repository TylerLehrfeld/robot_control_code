
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

GCon g = 0;
GCStringIn GALIL_IP_STRING = "192.168.1.10";

void stop_galil()
{
    if (g)         // don't call close on a nullptr
        GClose(g); // Don't forget to close!
}

void check(GReturn rc)
{
    if (rc != G_NO_ERROR)
    {
        std::cout << "ERROR:" << rc << std::endl;
        stop_galil();
        throw std::runtime_error("galil error");
    }
}
std::string to_string(double d)
{
    std::stringstream ss;
    ss << d;
    return ss.str();
}

void init_galil(int home_reho_or_updown)
{
    char buf[1024]; // traffic buffer

    check(GVersion(buf, sizeof(buf)));
    printf("version: %s\n", buf); // Print the library version

    char addresses[1024];
    GAddresses(addresses, sizeof(addresses));
    std::cout << "Devices found: " << addresses << std::endl;

    check(GOpen("192.168.1.10 --direct", &g)); // Open a connection to Galil, store the identifier in g.

    check(GInfo(g, buf, sizeof(buf)));
    printf("info: %s\n", buf); // Print the connection info

    check(GCommand(g, "MG TIME", buf, sizeof(buf), 0)); // Send MG TIME. Because response is ASCII, don't care about bytes read.
    printf("response: %s\n", buf);                      // Print the response
    if (home_reho_or_updown == 1)
        check(GProgramDownloadFile(g, "/home/amiro/Documents/Galil/main.dmc", "--max 4"));
    if (home_reho_or_updown == 2)
        check(GProgramDownloadFile(g, "/home/amiro/Documents/Galil/rc.dmc", "--max 4"));
    if (home_reho_or_updown == 3)
    {
        check(GProgramDownloadFile(g, "/home/amiro/Documents/Galil/test.dmc", "--max 4"));
        // char program[G_HUGE_BUFFER];
        // GProgramUpload(g, program, G_HUGE_BUFFER);
        // printf("%s\n", program);
    }
    if (home_reho_or_updown == 4)
    {
        check(GProgramDownloadFile(g, "/home/amiro/Documents/Galil/spinA.dmc", "--max 4"));
    }
}

int GoToLowBlocking(double left, double right)
{
    char buf[G_SMALL_BUFFER]; // traffic buffer
    GSize read_bytes = 0;     // bytes read in GCommand
    int value = 0;
    GCommand(g, ("tgtMmB = " + to_string(right)).c_str(), buf, G_SMALL_BUFFER, &read_bytes);
    GCommand(g, ("tgtMmE = " + to_string(left)).c_str(), buf, G_SMALL_BUFFER, &read_bytes);
    GCommand(g, "XQ #GoToLow, 7", buf, G_SMALL_BUFFER, &read_bytes);
    std::this_thread::sleep_for(100ms);
    do
    {
        GCmdI(g, "dMotion = ?", &value);
        std::this_thread::sleep_for(500ms);
    } while (!value);
    std::this_thread::sleep_for(1750ms);
    return 0;
}

int GoToBothBlocking(slider_positions sliders) {
    char buf[G_SMALL_BUFFER];
    char buf2[G_SMALL_BUFFER]; // traffic buffer
    GSize read_bytes = 0;
    GSize read_bytes2 = 0;     // bytes read in GCommand
    int valueU = 0;
    int valueL = 0;
    GCommand(g, ("tgtMmB = " + to_string(sliders.right_middle_slider_y)).c_str(), buf, G_SMALL_BUFFER, &read_bytes);
    GCommand(g, ("tgtMmE = " + to_string(sliders.left_middle_slider_y)).c_str(), buf, G_SMALL_BUFFER, &read_bytes);
    GCommand(g, ("tgtMmA = " + to_string(sliders.right_slider_y)).c_str(), buf, G_SMALL_BUFFER, &read_bytes);
    GCommand(g, ("tgtMmF = " + to_string(sliders.left_slider_y)).c_str(), buf, G_SMALL_BUFFER, &read_bytes);
    GCommand(g, "XQ #GoToLow, 3", buf2, G_SMALL_BUFFER, &read_bytes2);
    GCommand(g, "XQ #GoToUp, 7", buf, G_SMALL_BUFFER, &read_bytes);
    std::this_thread::sleep_for(100ms);
    do
    {
        GCmdI(g, "dMotionL = ?", &valueL);
        GCmdI(g, "dMotionU = ?", &valueU);

        std::this_thread::sleep_for(500ms);
    } while (!(valueL && valueU));
    std::this_thread::sleep_for(1750ms);
    return 0;
}

int GoToUpBlocking(double left, double right)
{
    char buf[G_SMALL_BUFFER]; // traffic buffer
    GSize read_bytes = 0;     // bytes read in GCommand
    int value = 0;
    GCommand(g, ("tgtMmA = " + to_string(right)).c_str(), buf, G_SMALL_BUFFER, &read_bytes);
    GCommand(g, ("tgtMmF = " + to_string(left)).c_str(), buf, G_SMALL_BUFFER, &read_bytes);
    GCommand(g, "XQ #GoToUp, 7", buf, G_SMALL_BUFFER, &read_bytes);
    std::this_thread::sleep_for(300ms);
    do
    {
        GCmdI(g, "dMotion = ?", &value);
        std::this_thread::sleep_for(500ms);
    } while (!value);
    std::this_thread::sleep_for(1750ms);
    return 0;
}

int GoToPosBlocking(double left, double right)
{
    char buf[G_SMALL_BUFFER]; // traffic buffer
    GSize read_bytes = 0;     // bytes read in GCommand
    int value = 0;
    GCommand(g, ("tgtMmB = " + to_string(right)).c_str(), buf, G_SMALL_BUFFER, &read_bytes);
    GCommand(g, ("tgtMmE = " + to_string(left)).c_str(), buf, G_SMALL_BUFFER, &read_bytes);
    GCommand(g, "XQ #GoToPos, 7", buf, G_SMALL_BUFFER, &read_bytes);
    std::this_thread::sleep_for(100ms);
    do
    {
        GCmdI(g, "dMotion = ?", &value);
        std::this_thread::sleep_for(500ms);
    } while (!value);
    std::this_thread::sleep_for(1750ms);
    return 0;
}

int HomeUpBlocking(bool msideA, bool msideF)
{
    char buf[G_SMALL_BUFFER]; // traffic buffer
    GSize read_bytes = 0;     // bytes read in GCommand
    int value = 0;
    GCommand(g, ("msideA = " + to_string((int)msideA)).c_str(), buf, G_SMALL_BUFFER, &read_bytes);
    GCommand(g, ("msideF = " + to_string((int)msideF)).c_str(), buf, G_SMALL_BUFFER, &read_bytes);
    GCommand(g, "XQ #homeUp, 0", buf, G_SMALL_BUFFER, &read_bytes);
    std::this_thread::sleep_for(100ms);
    do
    {
        GCmdI(g, "dHome = ?", &value);
        std::this_thread::sleep_for(500ms);
    } while (!value);
    std::this_thread::sleep_for(1750ms);

    return 0;
}

int HomeLowBlocking(bool msideB, bool msideE)
{
    char buf[G_SMALL_BUFFER]; // traffic buffer
    GSize read_bytes = 0;     // bytes read in GCommand
    int value = 0;
    GCommand(g, ("msideB = " + to_string((int)msideB)).c_str(), buf, G_SMALL_BUFFER, &read_bytes);
    GCommand(g, ("msideE = " + to_string((int)msideE)).c_str(), buf, G_SMALL_BUFFER, &read_bytes);
    GCommand(g, "XQ #homeLow, 0", buf, G_SMALL_BUFFER, &read_bytes);
    std::this_thread::sleep_for(100ms);
    do
    {
        GCmdI(g, "dHome = ?", &value);
        std::this_thread::sleep_for(500ms);
    } while (!value);
    std::this_thread::sleep_for(1750ms);
    return 0;
}

int move_slider_A(int mm)
{
    char buf[G_SMALL_BUFFER]; // traffic buffer
    GSize read_bytes = 0;     // bytes read in GCommand
    int value = 0;
    std::string string = "tgtMmA = " + mm; 
    const char* str = string.c_str();
    printf("%s\n", str);
    GCommand(g, str, buf, G_SMALL_BUFFER, &read_bytes);
    GCommand(g, "XQ #spinA, 1", buf, G_SMALL_BUFFER, &read_bytes);
    std::this_thread::sleep_for(100ms);
    do
    {

        GCmdI(g, "aFIN = ?", &value);
        std::this_thread::sleep_for(500ms);
    } while (!value);
    std::this_thread::sleep_for(1750ms);
    return 0;
}

/**
 * @brief use the slider positions to move the robot into a desired position
 *
 * @param positions
 */
void move_robot_with_slider_positions(slider_positions positions)
{
    std::string command;
    char buf[G_SMALL_BUFFER];
    GSize read_bytes = 0;
    double right = BASE_TO_SLIDER_MAX - positions.right_slider_y - HALF_SLIDER_WIDTH;
    double left = BASE_TO_SLIDER_MAX - positions.left_slider_y - HALF_SLIDER_WIDTH;
    double right_middle = BASE_TO_SLIDER_MAX - positions.right_middle_slider_y - HALF_SLIDER_WIDTH;
    double left_middle = BASE_TO_SLIDER_MAX - positions.left_middle_slider_y - HALF_SLIDER_WIDTH;
    std::cout << left << " " << left_middle << " " << right_middle << " " << right << std::endl;
    std::cout << "continue with motion?" << std::endl;
    // std::cin >> command;
    std::cout << "beginning motion" << std::endl;
    GoToLowBlocking(left_middle, right_middle);
    GoToUpBlocking(left, right);
    //GoToBothBlocking(positions);
    //GoToPosBlocking(left_middle, right_middle);
    // GoToUpBlocking(upper_left, upper_right);
    std::cout << "finished motion" << std::endl;

    /*std::stringstream ss;
    ss << "tgtMmA = " << right;
    GCStringIn command = ss.str().c_str();
    ss.clear();
    GCommand(g, command, buf, G_SMALL_BUFFER, &read_bytes);
    ss << "tgtMmF = " << left;
    command = ss.str().c_str();
    GCommand(g, command, buf, G_SMALL_BUFFER, &read_bytes);
    //ss.clear();
    //ss << "tgtMmB = " << right_middle;
    //command = ss.str().c_str();
    //GCommand(g, command, buf, G_SMALL_BUFFER, &read_bytes);
    //ss.clear();
    //ss << "tgtMmE = " << left_middle;
    //command = ss.str().c_str();
    //GCommand(g, command, buf, G_SMALL_BUFFER, &read_bytes);
    //GCommand(g, "XQ #GoToLow, 1", buf, G_SMALL_BUFFER, &read_bytes);
    GCommand(g, "XQ #GoToUp, 1", buf, G_SMALL_BUFFER, &read_bytes);
    std::this_thread::sleep_for(100ms);
    int value = 0;
    do {
        GCmdI(g, "dMotion = ?", &value);
        std::this_thread::sleep_for(500ms);
    } while(!value);
    std::this_thread::sleep_for(1750ms);*/
}


#endif