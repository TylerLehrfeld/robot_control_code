#include <fstream>
#include "NewTransform.h"
#include "Matrix.h"
#include "kinematic_structs.h"


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

NewTransform parse_beginning(std::ifstream& file) {
 get_transform(file,1);
 NewTransform T = get_transform(file, 2);
 get_transform(file,1);
 return T;
 
}

int main() {
    std::ifstream results("resultsy.txt");
    std::ifstream grid_file("grid.txt");

    if(!grid_file.is_open()) {
        std::cout << "grid file not open";
        return -1;
    }
    int num_results = 67;
    std::string line;
    NewTransform F_OM2_home = parse_beginning(results);
    Matrix z(3,1,{0,0,1});
    Matrix up = (F_OM2_home.to_transform().R_AB * z);
    NewTransform prev_F_RN_measured;
    NewTransform prev_F_RN_expected;
    NewTransform prev_F_OM2;
    Point prev_expected_needle;
    Point prev_measured_needle;
    double prev_y = 0;
    NewTransform F_M2N(0,0,M_PI,0.0015843, 28.1358, -5.656892);
    Point zero = {0,0,0};
    for(int i = 0; i < num_results; i++) {
        std::string line;
        grid_file >> line;
        if(line == "") {
            break;
        }
        std::string x = line.substr(1,line.find(",")-1);
        std::string y = line.substr(line.find(",")+1, line.length() - line.find(",") -2);
        //std::cout << x << ", " << y << std::endl;
        double cur_y = std::stod(y);
        results >> line;
        results >> line;
        NewTransform F_OM1 = get_transform(results, 1);
        NewTransform F_OM2 = get_transform(results, 1);
        NewTransform F_RN_measured = get_transform(results, 2);
        NewTransform F_RN_expected = get_transform(results, 2);
        Point measured_base = F_RN_measured * zero;
        //Point expected_base = F_RN_expected * zero;
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
        Point needle = LOWER_END_EFFECTOR_TO_NEEDLEPOINT;
        needle.z = -115;
        Point sec = {102.515, 23.8609, 811.329};
        if(i != -1) {
            //std::cout << prev_y - cur_y << std::endl;
            (F_OM2 * F_M2N * needle);
            //Point p_0 = (prev_F_OM2 * zero);
            Point p = (F_OM2 *F_M2N* zero) - sec;
            std::cout << cur_y -410 << ", " << p.magnitude() << ";" << std::endl;
            //p.print_desmos();
            //(p - p_0).print_desmos();
            if(prev_y - cur_y > 0) {
                //std::cout << "(" << cur_y << ","<< abs((p - p_0).magnitude() - abs(prev_y - cur_y))<<")" <<std::endl;
                 
            }
            //std::cout << (p - p_0).magnitude() << std::endl;
            //acos(((F_OM2.to_transform().R_AB * z).transpose() * up).magnitude());

        }
        //std::cout << acos(((F_OM2.to_transform().R_AB * z).transpose() * up).magnitude()) << std::endl;
        prev_F_OM2 = F_OM2;
        prev_y = cur_y;
        //prev_F_RN_measured = F_RN_measured;
        //prev_F_RN_expected = F_RN_expected;
        //prev_expected_needle = expected_needle;
        //prev_measured_needle = measured_needle;
    }
}