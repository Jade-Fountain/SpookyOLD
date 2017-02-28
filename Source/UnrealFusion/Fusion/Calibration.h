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
		//----------------
		//PRIVATE MEMBERS
		//----------------

		//Difference threshold: store new measurement if difference to last measurement is larger than this
		float diff_threshold = 0.5;
		//Count Threshold: Calibrate when this many samples acquired
		int count_threshold = 10;

		//Table for looking up data relevant to determining transforms
		CalibrationDataSet calibrationSet;

		//Data for resulting calibrations
		std::map<SystemPair, CalibrationResult> calibrationResults;
		
		//----------------
		//PRIVATE METHODS
		//----------------

		//Checks there is data corresponding to more than one system for a given node in a measurement group
		std::vector<std::pair<Measurement::Ptr, NodeDescriptor>> filterLonelyData(const std::vector<std::pair<Measurement::Ptr, NodeDescriptor>>& measurementQueue);

		//Returns true if sufficient movement has occurred to warrant recording of data
		bool checkChanges(const std::vector<std::pair<Measurement::Ptr, NodeDescriptor>>& measurements);

		//Calibrate two particular data streams
		CalibrationResult calibrateStreams(const std::vector<Measurement::Ptr>& measurements1, const std::vector<Measurement::Ptr>& measurements2);

		//Different types of calibration:
		CalibrationResult calPosPos(const std::vector<Measurement::Ptr>& measurements1, const std::vector<Measurement::Ptr>& measurements2);

	public:
		//Add data for later calibration
		void addMeasurement(const Measurement::Ptr& m, const NodeDescriptor& node);
		void addMeasurementGroup(const std::vector<std::pair<Measurement::Ptr, NodeDescriptor>>& measurementQueue);

		//Calibrate
		void calibrate();

		//Searches for calibration results and returns them for use in fusion
		CalibrationResult getResultsFor(SystemDescriptor s1, SystemDescriptor s2);
	};

}
