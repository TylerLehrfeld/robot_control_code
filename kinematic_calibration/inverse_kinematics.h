
#include "kinematics.h"

template<typename T>
Thetas<T> get_thetas(Transform<T> EE_transform, Parameters<T> parameters) {
	T e_diff = parameters.tunable_params.top_needle_to_holder_distance - parameters.tunable_params.bottom_needle_to_holder_distance;

	Point<T> E2= 
}
