

#include "kinematics.h"
#include "templated_classes/Templated_Point.h"
#include "templated_classes/Templated_Transform.h"


enum atracsys_bitmask {
	BOTH,
	FOM1,
	FOM2
};

Measurement<double> get_atracsys_measurement(enum atracsys_bitmask);

