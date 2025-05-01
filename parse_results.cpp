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
    std::ifstream grid_file("grid.txt");

    if(!grid_file.is_open()) {
        std::cout << "grid file not open";
        return -1;
    }
    int num_results = 80;
    std::string line;
    parse_beginning(results);
    NewTransform prev_F_RN_measured;
    NewTransform prev_F_RN_expected;
    NewTransform prev_F_OM2;
    Point prev_expected_needle;
    Point prev_measured_needle;
    double prev_y = 0;
    Point zero = {.00558085, 28.3789, -5.49509};
    for(int i = 0; i < num_results; i++) {
        std::string line;
        grid_file >> line;
        if(line == "") {
            break;
        }
        std::string x = line.substr(1,line.find(",")-1);
        std::string y = line.substr(line.find(",")+1, line.length() - line.find(",") -2);
        std::cout << x << ", " << y << std::endl;
        double cur_y = std::stod(y);
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
            std::cout << prev_y - cur_y << std::endl;
            Point p_0 = (prev_F_OM2 * zero);
            Point p = (F_OM2 * zero);
            //p.print_desmos();
            (p - p_0).print_desmos();
            std::cout << (p - p_0).magnitude() << std::endl;

        }
        prev_F_OM2 = F_OM2;
        prev_y = cur_y;
        //prev_F_RN_measured = F_RN_measured;
        //prev_F_RN_expected = F_RN_expected;
        //prev_expected_needle = expected_needle;
        //prev_measured_needle = measured_needle;
    }
}