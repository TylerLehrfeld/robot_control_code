#include "galil_control_calls.h"

int main() {
    
    init_galil(3);
    GoToLowBlocking(25, 25);
    stop_galil();

}