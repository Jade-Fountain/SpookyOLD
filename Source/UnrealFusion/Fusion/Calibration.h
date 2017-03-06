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
			//Stores sensor samples per ID
			std::map<SensorID, std::vector<Measurement::Ptr>> sensors;
			//Adds a measurement to the stream
			void addMeasurement(const Measurement::Ptr& m);
			//Gets the size of the largest sensor stream and the corresponding sensor ID
			std::pair<SensorID, size_t> maxCount();
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
		float diff_threshold = 0.1;
		//Count Threshold: Calibrate when this many samples acquired
		int count_threshold = 50;

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

		//Calibrate with no prior knowledge available
		void calibrateInitial(SystemDescriptor system1, SystemDescriptor system2);

		//Calibrate two particular data streams
		CalibrationResult calibrateStreams(const std::vector<Measurement::Ptr>& measurements1, const std::vector<Measurement::Ptr>& measurements2);

		//Different types of calibration:
		CalibrationResult calPosPos(const std::vector<Measurement::Ptr>& measurements1, const std::vector<Measurement::Ptr>& measurements2);
		CalibrationResult calTT(const std::vector<Measurement::Ptr>& m1, const std::vector<Measurement::Ptr>& m2);

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
