#include "kinematic_structs.h"
#include <cmath>
#include <fstream>
#include <iostream>
#include <ostream>
#include "forward_kinematics.cpp"


int main() {
    std::ofstream output_csv;
    output_csv.open("output.csv");
    float left_slider_y = 106.565;
    float left_middle_slider_y =  BASE_TO_SLIDER_MAX-34.5634;
    float right_middle_slider_y =BASE_TO_SLIDER_MAX-29.8236;
    float right_slider_y = 106.565;
    
    int prev_count = 0;
    Point end_effector = get_end_effector(
        {-63, left_slider_y, 0}, {-21, left_middle_slider_y, 0},
        {21.07, right_middle_slider_y, 0}, {63, right_slider_y, 0},
        UPPER_BASE, LOWER_BASE, 0);
    end_effector.print();

    /**
     * Uncomment the two loops for the slider lengths you want to find the ideal slider length for according to your condition
     * 
     */
    //for(left_slider_y = 100; left_slider_y < 120; left_slider_y += .03) {
        for(left_middle_slider_y = BASE_TO_SLIDER_MIN+HALF_SLIDER_WIDTH; left_middle_slider_y < BASE_TO_SLIDER_MAX - HALF_SLIDER_WIDTH; left_middle_slider_y += .03) {
            for(right_middle_slider_y = BASE_TO_SLIDER_MIN + HALF_SLIDER_WIDTH; right_middle_slider_y < BASE_TO_SLIDER_MAX - HALF_SLIDER_WIDTH; right_middle_slider_y += .03) {
                //for(right_slider_y = 100; right_slider_y < 120; right_slider_y += .03) {
                    Point end_effector = get_end_effector(
                        {-63, left_slider_y, 0}, {-21, left_middle_slider_y, 0},
                        {21.07, right_middle_slider_y, 0}, {63, right_slider_y, 0},
                        UPPER_BASE, LOWER_BASE, 0);
                    if(highest_count != prev_count) {
                        output_csv << (BASE_TO_SLIDER_MAX) - left_slider_y <<", " <<(BASE_TO_SLIDER_MAX) - left_middle_slider_y<< ", " <<(BASE_TO_SLIDER_MAX) - right_middle_slider_y << ", "<< (BASE_TO_SLIDER_MAX) - right_slider_y << std::endl;    
                        prev_count = highest_count;
                    }
                }
            }
        //}
    //}

    //end_effector.print();
    output_csv.close();
}