#pragma once

#include <string>
#include <vector>
#include <map>
#include "Eigen/Core"
#include "FusionTypes.h"


namespace fusion {

	//Encapsulation for storing measurements	
	class CalibrationDataSet {

		//Encapsulation for accessing measurements corresponding to 
		class CalibrationData {
		private:
			//Stores sensor samples per ID
			std::map<SensorID, std::vector<Measurement::Ptr>> sensors;
		public:

		};

	private:
		//Stores the data for each System and each node which has sensors from that system
		std::map<SystemNodePair, CalibrationData> data;
	public:
		void addMeasurement(const Measurement::Ptr& m, const SystemDescriptor& system, const NodeDescriptor& node);

	};


	class Calibrator {
	private:
		
		//Table for looking up data relevant to determining transforms
		CalibrationDataSet calibrationSet;

		//Storage of output data
		std::map<SystemPair, CalibrationResult> results;

	public:

		void addMeasurement(const Measurement::Ptr& m, const NodeDescriptor& node);
	};

}
