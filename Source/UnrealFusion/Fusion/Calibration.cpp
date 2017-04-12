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
#include "UnrealFusion.h"
#include "Calibration.h"
#include "Logging.h"
#include "Fusion/Utilities/Conventions.h"
#include "Fusion/Utilities/TimeProfiling.h"

namespace fusion {
	/////////////////////////////////////////////////////////////////////////////////////////////////////////
	//									CalibrationDataSet
	/////////////////////////////////////////////////////////////////////////////////////////////////////////

	//-------------------------------------------------------------------------------------------------------
	//									Stream
	//-------------------------------------------------------------------------------------------------------

	void CalibrationDataSet::Stream::addMeasurement(const Measurement::Ptr& m) {
		utility::safeAccess(sensors, m->getSensorID()).push_back(m);
		while (sensors[m->getSensorID()].size() > max_samples) {
			//Erase oldest data
			sensors[m->getSensorID()].erase(sensors[m->getSensorID()].begin());
		}
	}

	std::pair<SensorID, size_t> CalibrationDataSet::Stream::maxCount()
	{
		std::pair<SensorID, size_t> result(0, 0);
		for (auto& sensor : sensors) {
			size_t s = sensor.second.size();
			if (result.second < s) {
				result.second = s;
				result.first = sensor.first;
			}
		}
		return result;
	}

	int CalibrationDataSet::Stream::totalCount()
	{
		int result = 0;
		for (auto& sensor : sensors) {
			result += sensor.second.size();
		}
		return result;
	}

	//-------------------------------------------------------------------------------------------------------
	//									CalibrationDataSet Members
	//-------------------------------------------------------------------------------------------------------


	void CalibrationDataSet::addMeasurement(const Measurement::Ptr& m, const SystemDescriptor& system, const NodeDescriptor& node) {
		SystemNodePair sysNode = SystemNodePair(system, node);
		systemNodeTable[sysNode].addMeasurement(m);
		//Store info for later
		systems.insert(system);
		nodes.insert(node);
	}

	float CalibrationDataSet::compareMeasurement(const Measurement::Ptr & m, const SystemDescriptor & system, const NodeDescriptor & node)
	{
		SystemNodePair sysNode = SystemNodePair(system, node);
		std::vector<Measurement::Ptr>& stream = utility::safeAccess(systemNodeTable[sysNode].sensors, m->getSensorID());
		//If no previous recorded data, return max difference
		if (stream.size() == 0) {
			return float(std::numeric_limits<float>::max());
		}
		//Otherwise compare the measurements
		return stream.back()->compare(m);
	}



	/////////////////////////////////////////////////////////////////////////////////////////////////////////
	//									Calibrator:Private
	/////////////////////////////////////////////////////////////////////////////////////////////////////////

	std::vector<Measurement::Ptr>
	Calibrator::filterLonelyData(const std::vector<Measurement::Ptr>& measurementQueue) {
		//Init result
		std::vector<Measurement::Ptr> result;
		//Structure for counting systems per node
		utility::SafeMap<NodeDescriptor, std::set<SystemDescriptor>> systemsPerNode;
		//utility::profiler.startTimer("Calibration: Filter - Count");

		//Count
		for (auto& m : measurementQueue) {
			systemsPerNode[m->getNode()].insert(m->getSystem());
		}
		//utility::profiler.endTimer("Calibration: Filter - Count");
		//Push back relevant measurments
		//utility::profiler.startTimer("Calibration: Filter - Pushback");
		for (auto& m : measurementQueue) {
			if (systemsPerNode[m->getNode()].size() > 1) {
				result.push_back(m);
			}
		}
		//utility::profiler.endTimer("Calibration: Filter - Pushback");
		return result;
	}

	bool Calibrator::checkChanges(const std::vector<Measurement::Ptr>& measurements) {
		//Check change for each measurement
		bool result = false;
		for (auto& mes : measurements) {
			NodeDescriptor node = mes->getNode();
			float diff = calibrationSet.compareMeasurement(mes, mes->getSystem(), node);
			//TODO:Perform next check over each node individually?

			//If any of the measurements are new then return true

			result = result || (diff > diff_threshold);
		}
		return result;
	}

	void Calibrator::calibrateSystems(SystemDescriptor system1, SystemDescriptor system2)
	{
		//Create key for the pair of systems
		SystemPair sysPair(system1, system2);

		//Initialise vectors of measurements relevant to calibrating system1 and system2
		std::vector<Measurement::Ptr> measurements1;
		std::vector<Measurement::Ptr> measurements2;

		getRelevantMeasurements(system1, system2, &measurements1, &measurements2, count_threshold[getResultsFor(system1,system2).state], false);

		//Calibrate
		if (measurements1.size() > 0) {
			CalibrationResult latestResult = getResultsFor(system1,system2);
			//latestResult.latency = estimateLatency(measurements1, measurements2);
			latestResult.latency = estimateLatency(measurements2, measurements1);
			Measurement::setLatencies(measurements1, latestResult.latency);
			FUSION_LOG("Estimated latency = " + std::to_string(latestResult.latency));

			//Resynchronise measurements after latency estimation
			getRelevantMeasurements(system1, system2, &measurements1, &measurements2, count_threshold[getResultsFor(system1, system2).state], true);
			calibrationResults[sysPair] = calibrateStreams(measurements1, measurements2, latestResult);

			//Debug
			//std::stringstream ss;
			//ss << "Results for X: " << system1.name << " --> " << system2.name << "(Combined nodes)\n" << calibrationResults[sysPair].transform.matrix() << "\n";
			//FUSION_LOG(ss.str());
		}
	}

	void Calibrator::getRelevantMeasurements(
		SystemDescriptor system1, 
		SystemDescriptor system2,
		std::vector<Measurement::Ptr>* measurements1,
		std::vector<Measurement::Ptr>* measurements2,
		int minMeasurementCount,
		bool clearMeasurementsWhenDone
	){
		//TODO:
		// - incorporate latency 

		//Loop through nodes and build up relevant measurements
		for (auto& node : calibrationSet.nodes) {

			//Keys for accessing data streams
			SystemNodePair sysNode1(system1, node);
			SystemNodePair sysNode2(system2, node);


			//If there is an entry for each system in the table, check if there is sufficient data for calibration
			if (calibrationSet.systemNodeTable.count(sysNode1) > 0 &&
				calibrationSet.systemNodeTable.count(sysNode2) > 0)
			{
				//Get maximum length of sensor stream
				int count1 = calibrationSet.systemNodeTable[sysNode1].totalCount();
				int count2 = calibrationSet.systemNodeTable[sysNode2].totalCount();

				//Streams of different length or not long enough- we cant use this data
				if (count1 < minMeasurementCount || count2 < minMeasurementCount) {
					continue; //cannot calibrate this pair of sensors yet
				}

				//Calibrate with complete bipartite graph of relationships
				for (auto& pair1 : calibrationSet.systemNodeTable[sysNode1].sensors) {
					SensorID id1 = pair1.first;
					std::vector<Measurement::Ptr>& m1_ = pair1.second;
					//Get measurements
					for (auto& pair2 : calibrationSet.systemNodeTable[sysNode2].sensors) {
						std::vector<Measurement::Ptr>& m2_ = pair2.second;

						//Synchronise the two streams
						std::vector<Measurement::Ptr> m1;
						std::vector<Measurement::Ptr> m2 = Measurement::synchronise(m2_,m1_,m1);

						measurements1->insert(measurements1->end(), m1.begin(), m1.end());
						measurements2->insert(measurements2->end(), m2.begin(), m2.end());

						//Clear the data used for calibration
						if (clearMeasurementsWhenDone) {
							m1_.clear();
							m2_.clear();
						}
					}
				}
			}
		}
	}


	CalibrationResult Calibrator::calibrateStreams(const std::vector<Measurement::Ptr>& m1, const std::vector<Measurement::Ptr>& m2, const CalibrationResult& calib)
	{
		//TODO: measurements are not sorted chronologically here
		Measurement::Type t1 = m1.front()->type;
		Measurement::Type t2 = m2.front()->type;
		//Bulk logic to route calibration procedures at runtime
		switch (t1) {
		case Measurement::Type::POSITION:
			if (t2 == Measurement::Type::POSITION || t2 == Measurement::Type::RIGID_BODY) {
				return calPos(m1, m2, calib);
			}
			break;
		case Measurement::Type::RIGID_BODY:
			if (t2 == Measurement::Type::RIGID_BODY) {
				return calPos(m1, m2, calib);
				//TODO: implement this:
				//return cal6DoF(m1, m2);
			}
			else if (t2 == Measurement::Type::POSITION) {
				return calPos(m1, m2, calib);
			}
		}
		FUSION_LOG("WARNING : no calibration model found for measurement types: " + std::to_string(t1) + " and " + std::to_string(t2));
		return CalibrationResult();
	}

	/////////////////////////////////////////////////////////////////////////////////////////////////////////
	//									Calibrator:Public
	/////////////////////////////////////////////////////////////////////////////////////////////////////////
	void Calibrator::addMeasurement(const Measurement::Ptr& m) {
		calibrationSet.addMeasurement(m, m->getSystem(), m->getNode());
	}

	void Calibrator::addMeasurementGroup(const std::vector<Measurement::Ptr>& measurementQueue) {
		//utility::profiler.startTimer("Calibration: 1");
		//Check there is data corresponding to more than one system for a given node, otherwise useless
		//TODO: optimise this filterLonelyData - currently takes way too long
		auto measurements = filterLonelyData(measurementQueue);
		//utility::profiler.endTimer("Calibration: 1");

		//Decide if data is useful
		//(if at least one stream has changed relative to previous measurements)
		//utility::profiler.startTimer("Calibration: 2");
		bool dataNovel = checkChanges(measurements);
		//utility::profiler.endTimer("Calibration: 2");

		if (dataNovel) {
			//Store the (refs to) the relevant measurements
			//FUSION_LOG("Adding calibration measurments!!");
			//utility::profiler.startTimer("Calibration: 3");
			for (auto& m : measurements) {
				addMeasurement(m);
			}
			//utility::profiler.startTimer("Calibration: 3");


		}
		//FUSION_LOG(utility::profiler.getReport());

	}

	void Calibrator::calibrate()
	{
		//For each unordered pairing of systems, check if there are common nodes
		for (std::set<SystemDescriptor>::iterator system1 = calibrationSet.systems.begin(); system1 != calibrationSet.systems.end(); system1++) {
			for (std::set<SystemDescriptor>::iterator system2 = std::next(system1); system2 != calibrationSet.systems.end(); system2++) {

				calibrateSystems(*system1, *system2);
				
			}
		}
	}

	bool Calibrator::isStable()
	{
		for(auto& cal : calibrationResults){
			if (!(cal.second.state == CalibrationResult::State::CALIBRATED)) return false;
		}
		return true;
	}

	CalibrationResult Calibrator::getResultsFor(SystemDescriptor s1, SystemDescriptor s2)
	{
		SystemPair forward(s1, s2);
		SystemPair reverse(s1, s2);
		if (calibrationResults.count(forward) > 0) {
			return calibrationResults[forward];
		}
		if(calibrationResults.count(reverse) > 0) {
			return calibrationResults[reverse].inverse();
		}
		CalibrationResult cr = CalibrationResult();
		cr.systems = forward;
		return cr;
	}

}
