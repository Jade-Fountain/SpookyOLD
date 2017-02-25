#pragma once

#include <string>
#include <map>
#include "Eigen/Core"
#include "Fusion/FusionTypes.h"
#include "Fusion/Calibration.h"

namespace fusion {

	class SensorSystem {

	};

	//Centre of a fusion plant
	class Core{

		//Calibration data per system pair (A,B) = std::pair<SystemDescriptor,SystemDescriptor>
		//sensorTransforms[(A,B)]: A -> B
		std::map<SystemPair, Calibration, SystemPairCompare> calibrations;
		std::map<SystemDescriptor, SensorSystem> systems;
	};

}