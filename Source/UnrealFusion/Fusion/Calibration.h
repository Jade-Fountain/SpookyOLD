#pragma once

#include <string>
#include <vector>
#include <map>
#include <set>
#include "Eigen/Core"
#include "FusionTypes.h"
#include "Utilities/DataStructures.h"


namespace fusion {

	//Encapsulation for storing measurements	
	class CalibrationDataSet {

		//Encapsulation for accessing measurements corresponding to 
		class Stream {
		public:
			//Stores sensor samples per ID
			SafeMap<SensorID, std::vector<Measurement::Ptr>> sensors;
			void addMeasurement(const Measurement::Ptr& m);
		};

	private:
		//Stores the data for each System and each node which has sensors from that system
		std::map<SystemNodePair, Stream> data;
	public:
		void addMeasurement(const Measurement::Ptr& m, const SystemDescriptor& system, const NodeDescriptor& node);
		float compareMeasurementToLatest(const Measurement::Ptr& m, const SystemDescriptor& system, const NodeDescriptor& node);

	};


	class Calibrator {
	private:
		//Difference threshold
		float threshold = 0.1;

		//Table for looking up data relevant to determining transforms
		CalibrationDataSet calibrationSet;

		//Storage of output data
		SafeMap<SystemPair, CalibrationResult> results;

		//Checks there is data corresponding to more than one system for a given node in a measurement group
		std::vector<std::pair<Measurement::Ptr, NodeDescriptor>> filterLonelyData(const std::vector<std::pair<Measurement::Ptr, NodeDescriptor>>& measurementQueue);

		//Returns true if sufficient movement has occurred to warrant recording of data
		bool checkChanges(const std::vector<std::pair<Measurement::Ptr, NodeDescriptor>>& measurements);
	public:
		//Add data for later calibration
		void addMeasurement(const Measurement::Ptr& m, const NodeDescriptor& node);
		void addMeasurementGroup(const std::vector<std::pair<Measurement::Ptr, NodeDescriptor>>& measurementQueue);

		//
	};

}
