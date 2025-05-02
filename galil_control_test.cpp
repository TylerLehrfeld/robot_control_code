#include "galil_control_calls.h"

int main() {
    
    init_galil(3);
    GoToUpBlocking(40, 40);
    stop_galil();
}