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
		private:
			//Stores sensor samples per ID
			SafeMap<SensorID, std::vector<Measurement::Ptr>> sensors;
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
		SafeMap<SystemPair, CalibrationResult> results;

		//Checks there is data corresponding to more than one system for a given node in a measurement group
		std::vector<std::pair<Measurement::Ptr, NodeDescriptor>> filterLonelyData(const std::vector<std::pair<Measurement::Ptr, NodeDescriptor>>& measurementQueue);
	public:
		//Add data for later calibration
		void addMeasurement(const Measurement::Ptr& m, const NodeDescriptor& node);
		void addMeasurementGroup(const std::vector<std::pair<Measurement::Ptr, NodeDescriptor>>& measurementQueue);

		//
	};

}
