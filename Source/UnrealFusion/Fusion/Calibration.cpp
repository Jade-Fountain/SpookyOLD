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
#include <math.h>

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

	std::string CalibrationDataSet::getStateSummary() {
		std::stringstream ss;
		std::string spacer = "               ";
		auto spaceString = [spacer](const std::string& s){
			std::string new_s = s + spacer.substr(0, std::max(int(spacer.size() - s.size()), 0)) + "|";
			return new_s;
		};
		//Initial newline and gap for columns
		ss << spacer << "|";
		//Column labels
		for (auto & sys : systems) {
			ss << spaceString(sys.name);
		}
		ss << std::endl;
		//Rows
		for (auto & node : nodes) {
			//Node name
			ss << spaceString(node.name);
			//For each col
			for (auto & sys : systems) {
				SystemNodePair sysNode(sys, node);
				//Create entry containing each sensor count
				std::stringstream entry;
				for (auto& sensor : systemNodeTable[sysNode].sensors) {
					entry << sensor.first << ":" << sensor.second.size() << ",";
				}
				ss << spaceString(entry.str());
			}
			ss << std::endl;
		}
		ss << "Key: (x:n) = sensor x has n stored measurements" << std::endl;
		return ss.str();
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

	std::map<NodeDescriptor, bool> Calibrator::checkChanges(const std::vector<Measurement::Ptr>& measurements) {
		//TODO: need to check that measurements aren't missing to:
		//	-Store sensors for each node. or sensors for each node. 


		//Check change for each measurement
		std::map<NodeDescriptor,bool> results;
		for (auto& mes : measurements) {
			NodeDescriptor node = mes->getNode();
			float diff = calibrationSet.compareMeasurement(mes, mes->getSystem(), node);

			//Result is true if all sensors on the given node exceed the threshold
			results[node] = utility::safeAccess(results, node, true) && (diff > diff_threshold);
		}
		//bool final_result = false;
		//for(auto& r : results){
		//	final_result = final_result || r.second;
		//}
		//If any nodes move then change has occured
		return results;
	}

	void Calibrator::calibrateSystems(SystemDescriptor system1, SystemDescriptor system2)
	{
		//Create key for the pair of systems
		SystemPair sysPair(system1, system2);

		//Initialise vectors of measurements relevant to calibrating system1 and system2
		std::vector<Measurement::Ptr> measurements1;
		std::vector<Measurement::Ptr> measurements2;

		int thres = count_threshold[getResultsFor(system1, system2).state];

		std::pair<int,int> count = countRelevantSynchronisedMeasurements(system1, system2, min_count_per_node);

		//Calibrate
		if (count.first > thres && count.second > thres) {
			//TODO:latency estimation

			CalibrationResult latestResult = getResultsFor(system1, system2);
			getRelevantMeasurements(system1, system2, &measurements1, &measurements2, min_count_per_node, true);
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
		int minCountPerNode,
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
				if (count1 < minCountPerNode || count2 < minCountPerNode) {
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
						std::vector<Measurement::Ptr> m1 = m1_;
						std::vector<Measurement::Ptr> m2 = m2_;
						//TODO: retarget high noise measurements, not high latency - I no longer know what I meant by this
						if (m1.size() < m2.size()) {
							Measurement::synchronise(m2, m1);
						}
						else if(m1.size() > m2.size()){
							Measurement::synchronise(m1, m2);
						}
						
						if (m1.size() > minCountPerNode && m2.size() > minCountPerNode)
						{
							measurements1->insert(measurements1->end(), m1.begin(), m1.end());
							measurements2->insert(measurements2->end(), m2.begin(), m2.end());
						}

					/*	std::stringstream ss;
						ss << "M1 = " << m1_.front()->getSensor()->system.name << " count = " << m1_.size() << std::endl;
						ss << "M2 = " << m2_.front()->getSensor()->system.name << " count = " << m2_.size() << std::endl;

						ss << "M1 retargeted = " << m1.front()->getSensor()->system.name << ", " << m1.front()->getSensor()->getNode().name << " final count = " << m1.size() << std::endl;
						ss << "M2 retargeted = " << m2.front()->getSensor()->system.name << ", " << m2.front()->getSensor()->getNode().name << " final count = " << m2.size() << std::endl;
						FUSION_LOG(ss.str());*/

						//Clear the data used for calibration
						//Clear data even if it isnt used because it is not synchronised
						if (clearMeasurementsWhenDone) {
							m1_.clear();
							m2_.clear();
						}
					}
				}
			}
		}
	}


	std::pair<int, int> Calibrator::countRelevantSynchronisedMeasurements(
		SystemDescriptor system1,
		SystemDescriptor system2,
		int minCountPerNode
	) {
		//TODO:
		// - incorporate latency 
		std::pair<int, int> count = std::make_pair(0, 0);

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
				if (count1 < minCountPerNode || count2 < minCountPerNode) {
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
						std::vector<Measurement::Ptr> m1 = m1_;
						std::vector<Measurement::Ptr> m2 = m2_;

						if (m1.size() < m2.size()) {
							Measurement::synchronise(m2, m1);
						}
						else if (m1.size() > m2.size()) {
							Measurement::synchronise(m1, m2);
						}

						//If there are sufficient measurements remaining, then add them to the count
						if (m1.size() > minCountPerNode && m2.size() > minCountPerNode)
						{
							count.first += m1.size();
							count.second += m2.size();
						}

					}
				}
			}
		}
		return count;
	}

	std::pair<int, int> Calibrator::countRelevantMeasurements(
		SystemDescriptor system1,
		SystemDescriptor system2,
		int minCountPerNode
	) {
		//TODO:
		// - incorporate latency 
		std::pair<int, int> count = std::make_pair(0, 0);

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
				if (count1 < minCountPerNode || count2 < minCountPerNode) {
					continue; //cannot calibrate this pair of sensors yet
				}

				count.first += count1;
				count.second += count2;
			}
		}
		return count;
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
				//return calPos(m1, m2, calib);
				//TODO: implement this:
				return cal6DoF(m1, m2, calib);
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
		std::map<NodeDescriptor, bool> dataNovel = checkChanges(measurements);
		//utility::profiler.endTimer("Calibration: 2");

		//Store the (refs to) the relevant measurements
		//FUSION_LOG("Adding calibration measurments!! " + std::to_string(measurements.size()));
		//utility::profiler.startTimer("Calibration: 3");
		for (auto& m : measurements) {
			if (dataNovel[m->getNode()]) {
				addMeasurement(m);
			}
		}
		//utility::profiler.startTimer("Calibration: 3");

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

	void  Calibrator::setResults(const CalibrationResult & r) {
		SystemPair forward = r.systems;
		SystemPair reverse(forward.second, forward.first);
		calibrationResults[forward] = r;
		calibrationResults[reverse] = r.inverse();
	}

	CalibrationResult Calibrator::getResultsFor(SystemDescriptor s1, SystemDescriptor s2) const
	{
		SystemPair forward(s1, s2);
		if (s1 == s2) {
			CalibrationResult cr = CalibrationResult();
			cr.systems = forward;
			return cr;
		}
		if (calibrationResults.count(forward) > 0) {
			return calibrationResults.at(forward);
		}
		SystemPair reverse(s2, s1);
		if(calibrationResults.count(reverse) > 0) {
			return calibrationResults.at(reverse).inverse();
		}
		CalibrationResult cr = CalibrationResult();
		cr.systems = forward;
		return cr;
	}

	std::string Calibrator::getStateSummary() {
		return calibrationSet.getStateSummary();
	}

}
