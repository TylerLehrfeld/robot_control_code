#include <string>
#include <iostream>
#include <fstream>
#include "NewTransform.h"
#include "galil_control_calls.h"
#include <string>
#include "./scripts/pivot_needle.h"
#include "./scripts/pivot_robot.h"
#include "inverse_kinematics.h"

NewTransform get_average_transform(vector<NewTransform> transforms) {
    deconstructed_transform tot;
    for(int i = 0; i < transforms.size(); i++) {
      tot = transforms[i].get_params() + tot;
    }
    tot.theta_x = tot.theta_x / transforms.size();
    tot.theta_y = tot.theta_y / transforms.size();
    tot.theta_z = tot.theta_z / transforms.size();
    tot.x = tot.x / transforms.size();
    tot.y = tot.y / transforms.size();
    tot.z = tot.z / transforms.size();
    return NewTransform(tot);
  };


int main() {
    while(true) {
        std::string command = "";
        if(command == "q") {
            break;
        } else {
            std::cout << "Begin needle marker calibration? After continuing, start pivoting the needle such that new optical frames can be read." << std::endl;
            std::cin >> command;
            std::cout << "getting transforms from out.txt" << std::endl;
            vector<NewTransform> F_M2Ns;
            do {
                NewTransform F_M2N = get_needle_pivot_transform();
                F_M2N.print();
                std::cout << "Begin needle marker calibration again (r: redo, a: add another, c: continue to robot pivot)? After continuing, start pivoting the needle such that new optical frames can be read" << std::endl;
                std::cin >> command;
                if(command == "a" || command == "c") {
                    F_M2Ns.push_back(F_M2N);
                }
            } while(command != "c");
            
            NewTransform F_M2N = get_average_transform(F_M2Ns); 
            vector<NewTransform> F_M1Rs;
            std::cout << "Begin robot base calibration? After continuing, start pivoting around the robot base." << std::endl;
            std::cin >> command;
            do {
                NewTransform F_M1R = get_robot_pivot_transform(F_M2N);
                F_M1R.print();
                std::cout << "Begin robot base calibration again? (r: redo, a: add another trial, c: continue to benchtop test) After continuing, start pivoting around the robot base. enter c to move on" << std::endl;
                std::cin >> command;
                if(command == "a" || command == "c") {
                    F_M1Rs.push_back(F_M1R);
                }
            } while(command != "c");
            NewTransform F_M1R = get_average_transform(F_M1Rs);
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