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
    init_galil();
    bool left_near, right_near;
    /*std::cout << "Homing low. Enter 1 if the slider is behind the limit switch" << std::endl;
    std::cin >> left_near;
    std::cin >> right_near;
    HomeLowBlocking(left_near, right_near);
    std::cout << "Homing high. Enter 1 if the slider is behind the limit switch" << std::endl;
    std::cin >> left_near;
    std::cin >> right_near;
    HomeUpBlocking(left_near, right_near);*/
    
    std::ifstream grid_file("grid.txt");
    if(!grid_file.is_open()) {
        std::cout << "grid file not open";
        return -1;
    }
    std::string command = "";
    std::string x, y;
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
    while(true) {
        std::cin >> command;
        if(command == "q") {
            break;
        } else {
            Robot inverse_robot;
            std::string line;
            grid_file >> line;
            if(line == "") {
                break;
            }
            std::string x = line.substr(1,line.find(",")-1);
            std::string y = line.substr(line.find(",")+1, line.length() - line.find(",") -2);
            std::cout << x << ", " << y << std::endl;
            approach_definition def = {
                {std::stod(x), std::stod(y), -64.9},
                0,
                0
            };
            slider_positions position = inverse_kinematics(def, NewTransform(0,0,0,0,0,0),inverse_robot);
            
            
    
            move_robot_with_slider_positions(position);

            vector<NewTransform> F_OM1s;
            vector<NewTransform> F_OM2s;
            NewTransform F_OM1(0,0,0,0,0,0);
            NewTransform F_OM2(0,0,0,0,0,0);
            while(F_OM1s.size() < 5) {
                NewTransform newF1;
                read_transform("./out.txt", newF1, true);
                NewTransform newF2;
                read_transform("./out.txt", newF2, true); 
                if(!(newF1 == F_OM1 || newF2 == F_OM2)) {
                    F_OM1s.push_back(newF1);
                    F_OM1 = newF1;
                    F_OM2s.push_back(newF2);
                    F_OM2 = newF2;
                }
                delay_ms(500);
            }
            F_OM1 = get_average_transform(F_OM1s);
            F_OM2 = get_average_transform(F_OM2s);
            double rotation[3][3];
            Matrix rot(vector<Matrix>({inverse_robot.x_prime.to_matrix(), inverse_robot.y_prime.to_matrix(), inverse_robot.z_prime.to_matrix()}));
            Transform expected_F_RN(rot, inverse_robot.bottom_linkage.extended_end_effector.to_matrix());
            NewTransform New_expected_F_RN(expected_F_RN);
            NewTransform F_OM1_inverse = F_OM1.inverse();
            NewTransform actual_F_RN = F_M1R.inverse() * F_OM1_inverse * F_OM2 * F_M2N;
            NewTransform identity = New_expected_F_RN.inverse() * actual_F_RN;
            identity.get_params().print();
        }
        
    }
    return 0;
}