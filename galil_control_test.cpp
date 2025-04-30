#include "galil_control_calls.h"

int main() {
    
    init_galil(4);
    move_slider_A(15);
    stop_galil();

}