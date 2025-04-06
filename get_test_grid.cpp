const double HORIZONTAL_RANGE = 1000;
const double VERTICAL_RANGE = 1000;


#include "inverse_kinematics.h"
#include "kinematic_structs.h"
#include "Robot.h"
#include <fstream>

int main() {
    std::ofstream point_file;
    point_file.open("grid.txt");
    
    double no_extension_z = 0;
    double lowest_needle_extension = 1000;
    slider_positions sliders;
    for(double z = -70; z <= -60; z+=.1) {
        approach_definition approach = {{0,350, z}, 0, 0};
        Robot inverse_robot;
        sliders = inverse_kinematics(approach, NewTransform(0,0,0,0,0,0), inverse_robot);
        if(abs(inverse_robot.sliders.needle_extension) < abs(lowest_needle_extension)) {
            lowest_needle_extension = inverse_robot.sliders.needle_extension;
            std::cout << z << ": " << lowest_needle_extension  << " " << std::endl;
            
            no_extension_z = z;
        }
    }
    for(double x = -HORIZONTAL_RANGE/2; x <= HORIZONTAL_RANGE/2; x += 20) {
        for(double y = -VERTICAL_RANGE/2; y <= VERTICAL_RANGE/2; y +=20) {
            try {
                if(x == 0 && y == 200) {
                    std::cout << "here" << std::endl;
                }
                Robot inverse_robot;
                approach_definition approach = {{x,y, no_extension_z}, 0, 0};
                sliders = inverse_kinematics(approach, NewTransform(0,0,0,0,0,0), inverse_robot);
                std::string error_string;
                if(inverse_robot.is_valid(error_string)) {
                    point_file << "(" <<x <<"," <<y<<")" << std::endl;
                    //point_file << sliders.get_slider_string();
                }
            } catch (const std::runtime_error& e) {
                //std::cout << "caught" << std::endl;
            }
        }
    }
}