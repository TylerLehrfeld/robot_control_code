#include "galil_control_calls.h"

int main() {
    init_galil(1);
    HomeUpBlocking(1,1);
    stop_galil(); 
    
    init_galil(3);
    GoToUpBlocking(50,50);
    stop_galil();

}