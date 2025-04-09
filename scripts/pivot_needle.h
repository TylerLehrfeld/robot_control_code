#include "../Pivot.h"
#include <string>
#include <fstream>
#include <chrono>
#include <thread>
#include "../NewTransform.h"
#ifndef NEEDLE_PIVOT
#define NEEDLE_PIVOT

double read_transform(std::string filename, NewTransform& T, bool first) {
    std::ifstream file(filename);
    if(!file.is_open()) {
        std::cout << filename<<": file not open." << std::endl;
        return 0;
    }
    
    std::string transform_name;
    file >> transform_name;
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            std::string doubleStr;
            file >> doubleStr; 
            T.matrix[i][j] = stod(doubleStr);
        }
    }
    double registration_error;
    file >> registration_error;
    if(first) {
        return registration_error;
    } else {

        std::string transform_name;
        file >> transform_name;
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                std::string doubleStr;
                file >> doubleStr; 
                T.matrix[i][j] = stod(doubleStr);
            }
        }
        double registration_error;
        file >> registration_error;
    }
    file.close();
    return registration_error;
}

void delay_ms(int milliseconds) {
    std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
}


NewTransform get_needle_pivot_transform() {
    int num_measurements = 50;
    std::string file_path = "./out.txt";
    
    std::vector<Transform> transform_list;
    NewTransform old_transform;
    delay_ms(5000);
    while(transform_list.size() < num_measurements) {
        std::cout << transform_list.size() << std::endl;
        NewTransform T;
        read_transform(file_path, T, false);
        if(!(T == old_transform)) {
            transform_list.push_back(T.to_transform());
            T.print();
        }
        old_transform = T;
        delay_ms(200);
    }
    Pivot pivot_calibrator(transform_list);
    NewTransform Needle(0,0,0, pivot_calibrator.p_t.matrixArray[0], pivot_calibrator.p_t.matrixArray[1], pivot_calibrator.p_t.matrixArray[2]);
    return Needle;
}

#endif