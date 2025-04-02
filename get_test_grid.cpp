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
    slider_positions sliders;
    for(double z = -100; z <= 100; z+=1) {
        approach_definition approach = {{0,350, z}, 0, 0};
        Robot inverse_robot;
        sliders = inverse_kinematics(approach, Transform(0,0,0,0,0,0), inverse_robot);
        if(inverse_robot.sliders.needle_extension <= 1 && inverse_robot.sliders.needle_extension >= -1) {
            std::cout << z << ": " << inverse_robot.sliders.needle_extension << std::endl;
            no_extension_z = z;
        }
    }
    for(double x = -HORIZONTAL_RANGE/2; x <= HORIZONTAL_RANGE/2; x += 5) {
        for(double y = -VERTICAL_RANGE/2; y <= VERTICAL_RANGE/2; y +=5) {
            try {
                Robot inverse_robot;
                approach_definition approach = {{0,350, no_extension_z}, 0, 0};
                inverse_kinematics(approach, Transform(0,0,0,0,0,0), inverse_robot);
                if(inverse_robot.is_valid()) {
                    point_file << "(" <<x <<"," <<y<<")";
                }
            } catch (std::exception e) {
                
            }
        }
    }
}