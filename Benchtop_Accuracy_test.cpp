#include <string>
#include <iostream>
#include <fstream>
#include "NewTransform.h"
#include "galil_control_calls.h"
#include <string>
#include "./scripts/pivot_needle.h"
#include "./scripts/pivot_robot.h"
#include "inverse_kinematics.h"

//TODO: make this accurate or introduce calibration phase.
NewTransform F_M1R(0,0,0,0,0,0);
NewTransform F_M2N(0,0,0,0,0,0);

double read_transform(std::ifstream& file, NewTransform& T) {
    std::string transform_name;
    file >> transform_name;
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            file >> T.matrix[i][j];
        }
    }
    double registration_error;
    file >> registration_error;
    return registration_error;
}

int main() {
    while(true) {
        std::string command = "";
        std::cin >> command;
        if(command == "q") {
            break;
        } else {
            std::cout << "getting transforms from out.txt" << std::endl;
            
            std::cout << "Begin needle marker calibration? After continuing, start pivoting the needle such that new optical frames can be read." << std::endl;
            std::cin >> command;
            NewTransform F_M2N = get_needle_pivot_transform();
            std::cout << "Begin robot base calibration? After continuing, start pivoting around the robot base." << std::endl;
            std::cin >> command;
            NewTransform F_M1R = get_robot_pivot_transform(F_M2N);

            Robot inverse_robot;
            approach_definition def = {
                {0, 380, -64.9},
                0,
                0
            };
            slider_positions position = inverse_kinematics(def, NewTransform(0,0,0,0,0,0),inverse_robot);
            
            init_galil();
            
            move_robot_with_slider_positions(position);


            std::ifstream file("out.txt");
            if(!file.is_open()) {
                std::cout << "file is not open" << std::endl;
            }

            
            NewTransform F_OM1(0,0,0,0,0,0);
            double registration_error_1 = read_transform(file, F_OM1);
            NewTransform F_OM2(0,0,0,0,0,0);
            double registration_error_2 = read_transform(file, F_OM2);
            file.close();
            

        }
        
    }
    return 0;
}