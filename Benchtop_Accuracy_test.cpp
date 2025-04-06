#include <string>
#include <iostream>
#include <fstream>
#include "Transform.h"
#include <string>
//TODO: make this accurate or introduce calibration phase.
Transform F_M1R(0,0,0,0,0,0);
Transform F_M2N(0,0,0,0,0,0);

double read_transform(std::ifstream& file, Transform& T) {
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
            std::ifstream file("out.txt");
            if(!file.is_open()) {
                std::cout << "file is not open" << std::endl;
            }
            Transform F_OM1(0,0,0,0,0,0);
            double registration_error_1 = read_transform(file, F_OM1);
            Transform F_OM2(0,0,0,0,0,0);
            double registration_error_2 = read_transform(file, F_OM2);
            
            

        }
        
    }
    return 0;
}