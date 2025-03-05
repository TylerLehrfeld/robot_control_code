#include <iostream>
#include <string>

using std::string;


#include "3D_slicer_interface.h"

int main() {
    while(true) {
        approach_definition app;
        app = get_approach_from_3D_slicer_UI();
        
        string msg;
        std::cin >> msg;
        if(msg == "confirm") {
            
            break;
        }
    }
}