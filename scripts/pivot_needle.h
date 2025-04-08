#include "../PROGRAMS/Pivot.h"
#include <string>
#include <fstream>
#include <chrono>
#include <thread>
#include "../NewTransform.h"
#ifndef NEEDLE_PIVOT
#define NEEDLE_PIVOT

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

void delay_ms(int milliseconds) {
    std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
}


NewTransform get_needle_pivot_transform() {
    std::string file_path = "../out.txt";
    std::ifstream file(file_path);

    vector<Transform> transform_list;
    NewTransform old_transform;
    while(transform_list.size() < 30) {
        
        NewTransform T;
        read_transform(file, T);
        if(!(T == old_transform)) {
            transform_list.push_back(T.to_transform());
        }
        old_transform = T;
        delay_ms(300);
    }
    Pivot pivot_calibrator(transform_list);
    NewTransform Needle(0,0,0, pivot_calibrator.p_t.matrixArray[0], pivot_calibrator.p_t.matrixArray[1], pivot_calibrator.p_t.matrixArray[2]);
    return Needle;
}

#endif