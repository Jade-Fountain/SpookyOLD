#pragma once

#include <string>
#include <vector>
#include "Eigen/Core"
#include "Fusion/FusionTypes.h"



namespace fusion {

	class Calibrator {
	private:
		bool calibrated = false;

		//Transform maps
		Eigen::Matrix4f transform;

		//Data relevant to determining transform
		std::pair<std::vector<Measurement::Ptr>, std::vector<Measurement::Ptr>> measurements;
	
	public:


	};

}
