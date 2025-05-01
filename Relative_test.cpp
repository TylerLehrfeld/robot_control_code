#include <string>
#include <iostream>
#include <fstream>
#include "NewTransform.h"
#include "PointCloudTransform.h"
#include "galil_control_calls.h"
#include <string>
#include "./scripts/pivot_needle.h"
#include "inverse_kinematics.h"
#include "forward_kinematics.h"

int main() {
    std::string command;
    //delay_ms(5000);
    for(int i = 0; i < 20; i++) {
        //std::cout << "take measurement?" <<std::endl;
        //std::cin >> command;
        std::string file_path = "./out.txt";
        NewTransform F_OM2;
        NewTransform F_OM1;
        read_transform(file_path, F_OM1, true);
        read_transform(file_path, F_OM2, false);
        NewTransform F_M1M2 = F_OM1.inverse() * F_OM2;
        F_M1M2.print();
        F_M1M2.to_transform().p_AB.print_desmos();
        delay_ms(100);
    }
    
    
    
    
    
    
}