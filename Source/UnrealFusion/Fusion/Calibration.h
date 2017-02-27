#pragma once

#include <string>
#include <vector>
#include <map>
#include <set>
#include "Eigen/Core"
#include "FusionTypes.h"
#include "Utilities/DataStructures.h"
#include "Utilities/CalibrationUtilities.h"

namespace fusion {

	//Encapsulation for storing measurements	
	class CalibrationDataSet {
	public:

		//Encapsulation for accessing measurements corresponding to 
		class Stream {
		public:
			//Stores sensor samples per ID
			std::map<SensorID, std::vector<Measurement::Ptr>> sensors;
			//Adds a measurement to the stream
			void addMeasurement(const Measurement::Ptr& m);
			//Gets the size of the largest sensor stream and the corresponding sensor ID
			std::pair<SensorID, size_t> maxCount();
			//
			const std::vector<Measurement::Ptr>& get
		};


		//Stores the data for each System and each node which has sensors from that system
		//Picture it as a table of sensors with Systems naming the rows and Nodes naming the columns
		//Example:
		//			N1	N2	N3		...
		//		S1	s1	s2	s3,s4
		//		S2	r1	-	-
		//		S3	-	q1	-
		//
		std::map<SystemNodePair, Stream> systemNodeTable;
		std::set<SystemDescriptor> systems;
		std::set<NodeDescriptor> nodes;

		//Helper methods
		void addMeasurement(const Measurement::Ptr& m, const SystemDescriptor& system, const NodeDescriptor& node);
		float compareMeasurement(const Measurement::Ptr& m, const SystemDescriptor& system, const NodeDescriptor& node);

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

		//Calibrate
		void calibrate();
	};

}
