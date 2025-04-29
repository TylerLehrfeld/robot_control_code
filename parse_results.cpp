#include <fstream>
#include "NewTransform.h"
#include "Matrix.h"


NewTransform get_transform(std::ifstream& file, int num_strs) {
    std::string str = "";
    double num;
    int count = 0;
    while(str == "" || str == " " || count < num_strs) {
        file >> str;
        if(!(str == "" || str == " "))
            count++;
    }
    NewTransform T;
    for(int i = 0; i < 4; i++) {
        for(int j = 0; j < 4; j++) {
            file >> num;
            T.matrix[i][j] = num;
        }
        
    }
    return T;
}

Point get_point(std::ifstream& file, int num_strs) {
    Point p;
    std::string str = "";
    double num;
    int count = 0;
    while(str == "" || str == " " || count < num_strs) {
        file >> str;
        if(!(str == "" || str == " "))
            count++;
    }
    file >> p.x;
    file >> p.y;
    file >> p.z;
    return p;
    
}

void parse_beginning(std::ifstream& file) {
 get_transform(file,1);
 get_transform(file,1);

}

int main() {
    std::ifstream results("results.txt");
    int num_results = 21;
    std::string line;
    parse_beginning(results);
    NewTransform prev_F_RN_measured;
    NewTransform prev_F_RN_expected;
    Point prev_expected_needle;
    Point prev_measured_needle;
    Point zero = {0,0,0};
    for(int i = 0; i < num_results; i++) {
        results >> line;
        results >> line;
        NewTransform F_OM1 = get_transform(results, 1);
        NewTransform F_OM2 = get_transform(results, 1);
        NewTransform F_RN_measured = get_transform(results, 2);
        NewTransform F_RN_expected = get_transform(results, 2);
        Point measured_base = F_RN_measured * zero;
        Point expected_base = F_RN_expected * zero;
        //F_RN_measured.to_transform().p_AB.print_desmos();
        //F_RN_expected.to_transform().p_AB.print_desmos();
        Point expected_needle = get_point(results, 2);
        Point measured_needle = get_point(results, 2);
        Point diff = get_point(results, 2);
        results >> line;
        results >> line;
        double diff_mag;
        
        results >> diff_mag;
        //measured_needle.print_desmos();
        //expected_needle.print_desmos();
        if(i != 0) {
            Point p = (prev_F_RN_measured * zero);
            p.z = 0;
            Point adj_measured_base = measured_base;
            adj_measured_base.z = 0;
            (p - measured_base).print_desmos();
            std::cout << (p - measured_base).magnitude() << std::endl;
            (p - adj_measured_base).print_desmos();
            std::cout << (p - adj_measured_base).magnitude() << std::endl;
        }
        prev_F_RN_measured = F_RN_measured;
        prev_F_RN_expected = F_RN_expected;
        prev_expected_needle = expected_needle;
        prev_measured_needle = measured_needle;
    }
}