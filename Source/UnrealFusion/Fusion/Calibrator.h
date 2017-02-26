#pragma once

#include <string>
#include <vector>
#include <map>
#include "Eigen/Core"
#include "Fusion/FusionTypes.h"


namespace fusion {

	class CalibrationData {
		std::map<int, std::vector<Measurement::Ptr>> sensors;
	};

	class CalibrationDataSet {
	private:
		//Stores the data for each System and each node which has sensors from that system
		std::map<SystemNodePair, CalibrationData> data;
	public:

	};

	struct CalibrationResult {
		bool calibrated = false;
		Eigen::Matrix4f transform;
		float quality = 0;
	};

	class Calibrator {
	private:
		
		//Table for looking up data relevant to determining transforms
		CalibrationDataSet calibrationData;

		//Storage of output data
		std::map<SystemPair, CalibrationResult> results;

	public:


	};

}
