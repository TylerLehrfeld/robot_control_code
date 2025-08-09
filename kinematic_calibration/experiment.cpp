#include "../robot_controller.h"
#include <vector>

void get_positions(std::vector<slider_positions> &positions) {
 //TODO
}
static const slider_positions home_positions = {
    BASE_TO_SLIDER_MAX - 60,    //- HALF_SLIDER_WIDTH,
    BASE_TO_SLIDER_MAX - 26.79, // - HALF_SLIDER_WIDTH,
    BASE_TO_SLIDER_MAX - 23.48, //- HALF_SLIDER_WIDTH,
    BASE_TO_SLIDER_MAX - 60,    //- HALF_SLIDER_WIDTH,
    0};

int main() { 
	vector<slider_positions> positions;
	get_positions(positions);
	robot_controller rc(home_positions);

}

