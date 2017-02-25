#pragma once

#include <string>
#include <map>
#include "Eigen/Core"
#include "Fusion/FusionTypes.h"
#include "Fusion/Calibration.h"
#include "Fusion/FusionGraph.h"

namespace fusion {

	class SensorSystem {

	};

	//Centre of a fusion plant
	class Core{
		//Raw data ordered by sytem
		std::map<SystemDescriptor, SensorSystem> systems;

		//Calibration data per system pair (A,B) = std::pair<SystemDescriptor,SystemDescriptor>
		//sensorTransforms[(A,B)]: A -> B
		std::map<SystemPair, Calibration, SystemPairCompare> calibrations;

		//Fused data
		FusionGraph skeleton;
	};

}