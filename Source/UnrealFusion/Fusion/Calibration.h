#pragma once

#include <string>
#include <vector>
#include <map>
#include "Eigen/Core"
#include "FusionTypes.h"
#include "Utilities/DataStructures.h"


namespace fusion {

	//Encapsulation for storing measurements	
	class CalibrationDataSet {

		//Encapsulation for accessing measurements corresponding to 
		class Stream {
		private:
			//Stores sensor samples per ID
			MapToVector<SensorID, Measurement::Ptr> sensors;
		public:
			void addMeasurement(const Measurement::Ptr& m);
		};

	private:
		//Stores the data for each System and each node which has sensors from that system
		std::map<SystemNodePair, Stream> data;
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
		//Add data for later calibration
		void addMeasurement(const Measurement::Ptr& m, const NodeDescriptor& node);
		//
	};

}
