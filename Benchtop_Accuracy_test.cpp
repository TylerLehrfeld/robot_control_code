#include <string>
#include <iostream>
#include <fstream>
#include "NewTransform.h"
#include "galil_control_calls.h"
#include <string>
#include "./scripts/pivot_needle.h"
#include "./scripts/pivot_robot.h"
#include "inverse_kinematics.h"




int main() {
    while(true) {
        std::string command = "";
        if(command == "q") {
            break;
        } else {
            std::cout << "Begin needle marker calibration? After continuing, start pivoting the needle such that new optical frames can be read." << std::endl;
            std::cin >> command;
            std::cout << "getting transforms from out.txt" << std::endl;
            NewTransform F_M2N;
            do {
                F_M2N = get_needle_pivot_transform();
                F_M2N.print();
                std::cout << "Begin needle marker calibration again? After continuing, start pivoting the needle such that new optical frames can be read. Press c to continue on" << std::endl;
                std::cin >> command;
            } while(command != "c");
            NewTransform F_M1R;
            std::cout << "Begin robot base calibration? After continuing, start pivoting around the robot base." << std::endl;
            std::cin >> command;
            do {
                F_M1R = get_robot_pivot_transform(F_M2N);
                F_M1R.print();
                std::cout << "Begin robot base calibration again? After continuing, start pivoting around the robot base. enter c to move on" << std::endl;
                std::cin >> command;
                
            } while(command != "c");

            Robot inverse_robot;
            approach_definition def = {
                {0, 380, -64.9},
                0,
                0
            };
            slider_positions position = inverse_kinematics(def, NewTransform(0,0,0,0,0,0),inverse_robot);
            
            init_galil();
            
            move_robot_with_slider_positions(position);


            
            

            
            NewTransform F_OM1(0,0,0,0,0,0);
            double registration_error_1 = read_transform("./out.txt", F_OM1, true);
            NewTransform F_OM2(0,0,0,0,0,0);
            double registration_error_2 = read_transform("./out.txt", F_OM2, false);
            
            

        }
        
    }
    return 0;
}