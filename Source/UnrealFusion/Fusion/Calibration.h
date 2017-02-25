#pragma once

#include <string>
#include <vector>
#include "Eigen/Core"
#include "Fusion/FusionTypes.h"



struct Calibration {

	bool calibrated = false;

	//Transform maps
	Eigen::Matrix4f transform;

	std::pair<std::vector<Measurement>, std::vector<Measurement>> measurements;
};
