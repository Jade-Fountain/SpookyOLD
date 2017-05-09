/*  This file is part of UnrealFusion, a sensor fusion plugin for VR in the Unreal Engine
    Copyright (C) 2017 Jake Fountain
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
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
			//Upper bound for number of samples. 
			//Ideally shouldnt reach this, but included as safety
			int max_samples = 1000;
			//Stores sensor samples per ID
			std::map<SensorID, std::vector<Measurement::Ptr>> sensors;
			//Adds a measurement to the stream
			void addMeasurement(const Measurement::Ptr& m);
			//Gets the size of the largest sensor stream and the corresponding sensor ID
			std::pair<SensorID, size_t> maxCount();
			//Gets total sensor measurement count for this node and sensor
			int totalCount();
		};


		//Stores the data for each System and each node which has sensors from that system
		//Picture it as a table of sensor streams with Systems naming the rows and Nodes naming the columns
		//Example:
		//			N1	N2	N3		...
		//		S1	s1	s2	s3,s4
		//		S2	r1	-	-
		//		S3	-	q1	-
		//
		std::map<SystemNodePair, Stream> systemNodeTable;
		//Row labels:
		std::set<SystemDescriptor> systems;
		//Column labels:
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
		float diff_threshold = 0.5f;
		//TODO: change diff threshold for different calibration stages
		//{
		//	{ CalibrationResult::State::UNCALIBRATED, 0.1 },
		//	{ CalibrationResult::State::REFINING, 0.5 },
		//	{ CalibrationResult::State::CALIBRATED, 0.5 }
		//};

		//Count Threshold: Calibrate when this many samples acquired
		std::map<CalibrationResult::State, int> count_threshold = 
		{	
			{CalibrationResult::State::UNCALIBRATED,50},
			{CalibrationResult::State::REFINING,50},
			{CalibrationResult::State::CALIBRATED,50} 
		};

		//Table for looking up data relevant to determining transforms
		CalibrationDataSet calibrationSet;

		//Data for resulting calibrations
		std::map<SystemPair, CalibrationResult> calibrationResults;
		
		//----------------
		//PRIVATE METHODS
		//----------------

		//Checks there is data corresponding to more than one system for a given node in a measurement group
		std::vector<Measurement::Ptr> filterLonelyData(const std::vector<Measurement::Ptr>& measurementQueue);

		//Returns true if sufficient movement has occurred to warrant recording of data
		bool checkChanges(const std::vector<Measurement::Ptr>& measurements);

		//Calibrate two systems with respect to one another
		void calibrateSystems(SystemDescriptor system1, SystemDescriptor system2);

		//Gets the measurements relevant to calibration of system1 and system2
		//Returns an empty list if calibration not ready yet
		void getRelevantMeasurements(SystemDescriptor system1, 
									 SystemDescriptor system2, 
									 std::vector<Measurement::Ptr>* measurements1, 
									 std::vector<Measurement::Ptr>* measurements2, 
									 int minMeasurementCount,
									 bool clearMeasurementsWhenDone = true);

		//Calibrate two particular data streams
		CalibrationResult calibrateStreams(const std::vector<Measurement::Ptr>& m1, const std::vector<Measurement::Ptr>& m2, const CalibrationResult& calib);
		
		//----------------
		//CALIBRATION METHODS

		//Different types of calibration procedures:
		//see CalibrationProcedures.cpp for definitions
		//----------------

		//Calibrate two correlated positional measurements
		CalibrationResult calPos(const std::vector<Measurement::Ptr>& measurements1, const std::vector<Measurement::Ptr>& measurements2, const CalibrationResult& calib) const;

		//Calibrate two rigidly linked 6DoF sensors
		CalibrationResult cal6DoF(const std::vector<Measurement::Ptr>& m1, const std::vector<Measurement::Ptr>& m2) const;

		//Estimate latencies of multiple concatenated streams of measurements
		float estimateLatencies(const std::vector<Measurement::Ptr>& m1, const std::vector<Measurement::Ptr>& m2);

		//Returns the estimated latency l between two streams: m2[t] <-> m1[t+l]
		//Aka, m2 lags begind by l
		float estimateLatency(const std::vector<Measurement::Ptr>& m1, const std::vector<Measurement::Ptr>& m2);

	public:
		//Add data for later calibration
		void addMeasurement(const Measurement::Ptr& m);
		void addMeasurementGroup(const std::vector<Measurement::Ptr>& measurementQueue);

		//Calibrate
		void calibrate();

		//Returns true if useable data is now available
		bool isStable();

		//Searches for calibration results and returns them for use in fusion
		CalibrationResult getResultsFor(SystemDescriptor s1, SystemDescriptor s2);
	};

}
