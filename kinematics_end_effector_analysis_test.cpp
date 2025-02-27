#include "kinematics.cpp"
#include <fstream>
#include <iostream>

int main() {
    std::ofstream output_csv;
    output_csv.open("output.csv");
    float left_slider_y = 109.9;
    float left_middle_slider_y = 150.71;
    float right_middle_slider_y = 144.91;
    float right_slider_y = 109.9;
    float slider_y = 54.35 - .03;
    Point prev =  get_end_effector(
        {-63, left_slider_y, 0}, {-21, left_middle_slider_y, 0},
        {21.07, right_middle_slider_y, 0}, {63, slider_y, 0},
        {0, 186.4, 0}, {0, 223.62, 0}, 0);
    while(slider_y + 0.03 <= 166.65) {
        slider_y += 0.03;
        Point end_effector = get_end_effector(
            {-63, left_slider_y, 0}, {-21, left_middle_slider_y, 0},
            {21.07, right_middle_slider_y, 0}, {63, slider_y, 0},
            {0, 186.4, 0}, {0, 223.62, 0}, 0);
        output_csv << "" << slider_y << "," << (end_effector-prev).magnitude() << std::endl;
        prev = end_effector;
        
    }
    output_csv.close();
}