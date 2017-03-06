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
#include "Fusion/Utilities/CalibrationUtilities.h"
#include "Logging.h"
#include "Fusion/Utilities/Conventions.h"

namespace fusion {
	/////////////////////////////////////////////////////////////////////////////////////////////////////////
	//									CalibrationDataSet
	/////////////////////////////////////////////////////////////////////////////////////////////////////////

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

	std::vector<std::pair<Measurement::Ptr, NodeDescriptor>>
		Calibrator::filterLonelyData(const std::vector<std::pair<Measurement::Ptr, NodeDescriptor>>& measurementQueue) {
		//Init result
		std::vector<std::pair<Measurement::Ptr, NodeDescriptor>> result;
		//Structure for counting systems per node
		utility::SafeMap<NodeDescriptor, std::set<SystemDescriptor>> systemsPerNode;
		//Count
		for (auto& m : measurementQueue) {
			systemsPerNode[m.second].insert(m.first->getSystem());
		}
		//Push back relevant measurments
		for (auto& m : measurementQueue) {
			if (systemsPerNode[m.second].size() > 1) {
				result.push_back(m);
			}
		}
		return result;
	}

	bool Calibrator::checkChanges(const std::vector<std::pair<Measurement::Ptr, NodeDescriptor>>& measurements) {
		//Check change for each measurement
		bool result = false;
		for (auto& m : measurements) {
			auto& mes = m.first;
			auto& node = m.second;
			float diff = calibrationSet.compareMeasurement(mes, mes->getSystem(), node);
			//TODO:Perform next check over each node individually
			//If any of the measurements are new then return true

			result = result || (diff > diff_threshold);
		}
		return result;
	}

	void Calibrator::calibrateInitial(SystemDescriptor system1, SystemDescriptor system2)
	{
		//Create key for the pair of systems
		SystemPair sysPair(system1, system2);

		//Initialise vectors of measurements relevant to calibrating system1 and system2
		std::vector<Measurement::Ptr> measurements1;
		std::vector<Measurement::Ptr> measurements2;

		getRelevantMeasurements(system1, system2, &measurements1, &measurements2, initial_threshold);

		//Calibrate
		if (measurements1.size() > 0) {
			calibrationResults[sysPair] = calibrateStreams(measurements1, measurements2);

			//Debug
			std::stringstream ss;
			ss << "Results for X: " << system1.name << " --> " << system2.name << "(Combined nodes)\n" << calibrationResults[sysPair].transform.matrix() << "\n";
			FUSION_LOG(ss.str());
		}
	}

	void Calibrator::refineCalibration(SystemDescriptor system1, SystemDescriptor system2) {
		//Create key for the pair of systems
		SystemPair sysPair(system1, system2);

		//Initialise vectors of measurements relevant to calibrating system1 and system2
		std::vector<Measurement::Ptr> measurements1;
		std::vector<Measurement::Ptr> measurements2;

		getRelevantMeasurements(system1, system2, &measurements1, &measurements2, initial_threshold);

		//Calibrate
		if (measurements1.size() > 0) {
			//Do something
		}
	}

	void Calibrator::detectFaults(SystemDescriptor system1, SystemDescriptor system2) {
		//Create key for the pair of systems
		SystemPair sysPair(system1, system2);

		//Initialise vectors of measurements relevant to calibrating system1 and system2
		std::vector<Measurement::Ptr> measurements1;
		std::vector<Measurement::Ptr> measurements2;

		getRelevantMeasurements(system1, system2, &measurements1, &measurements2, initial_threshold);

		//Calibrate
		if (measurements1.size() > 0) {
			//Do something
		}
	}

	void Calibrator::getRelevantMeasurements(
		SystemDescriptor system1, 
		SystemDescriptor system2,
		std::vector<Measurement::Ptr>* measurements1,
		std::vector<Measurement::Ptr>* measurements2,
		int minMeasurementCount
	){

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
				std::pair<SensorID, size_t> max1 = calibrationSet.systemNodeTable[sysNode1].maxCount();
				std::pair<SensorID, size_t> max2 = calibrationSet.systemNodeTable[sysNode2].maxCount();

				//Streams of different length or not long enough- we cant use this data
				if (max1.second != max2.second || max1.second < minMeasurementCount || max2.second < minMeasurementCount) {
					//TODO: do something?
					continue; //cannot calibrate this pair of sensors yet
				}

				//Get measurements
				const auto& m1 = calibrationSet.systemNodeTable[sysNode1].sensors[max1.first];
				measurements1->insert(measurements1->end(), m1.begin(), m1.end());
				const auto& m2 = calibrationSet.systemNodeTable[sysNode2].sensors[max2.first];
				measurements2->insert(measurements2->end(), m2.begin(), m2.end());

				//Perform calibration
				//
				////Debug
				//std::stringstream ss;
				//ss << "Results for X: " << system1.name << " --> " << system2.name << "("<< node.name << ")\n" << results.back().transform.matrix() << "\n";
				//FUSION_LOG(ss.str());

				//Clear the data used for calibration
				calibrationSet.systemNodeTable[sysNode1].sensors[max1.first].clear();
				calibrationSet.systemNodeTable[sysNode2].sensors[max2.first].clear();
			}
		}
	}


	CalibrationResult Calibrator::calibrateStreams(const std::vector<Measurement::Ptr>& m1, const std::vector<Measurement::Ptr>& m2)
	{
		switch (m1.front()->type) {
		case MeasurementType::POSITION:
			if (m2.front()->type == MeasurementType::POSITION) {
				return calPosPos(m1, m2);
			}
			break;
		case MeasurementType::RIGID_BODY:
			if (m2.front()->type == MeasurementType::RIGID_BODY) {
				return calTT(m1, m2);
			}
		}
		FUSION_LOG("WARNING : no calibration model found for measurement types: " + std::to_string(m1.front()->type) + " and " + std::to_string(m2.front()->type));
		return CalibrationResult();
	}

	CalibrationResult Calibrator::calPosPos(const std::vector<Measurement::Ptr>& m1, const std::vector<Measurement::Ptr>& m2)
	{
		std::vector<Eigen::Vector3f> pos1(m1.size());
		std::vector<Eigen::Vector3f> pos2(m2.size());
		for (int i = 0; i < m1.size(); i++) {
			pos1[i] = m1[i]->getData();
			pos2[i] = m2[i]->getData();
		}
		CalibrationResult result;
		result.systems = SystemPair(m1.front()->getSystem(), m2.front()->getSystem());
		//Compute transform and error
		result.transform = utility::calibration::Position::calibrateIdenticalPair(pos1, pos2, &result.error);
		//TODO: compute quality
		result.quality = utility::calibration::qualityFromError(result.error,1);
		result.state = CalibrationResult::State::REFINING;
		return result;
	}

	CalibrationResult Calibrator::calTT(const std::vector<Measurement::Ptr>& m1, const std::vector<Measurement::Ptr>& m2)
	{
		//At least one node
		std::vector<std::vector<Eigen::Matrix4f>> pos1(1);
		std::vector<std::vector<Eigen::Matrix4f>> pos2(1);
		std::map<NodeDescriptor, int> nodes;
		nodes[m1.front()->getNode()] = 0;
		for (int i = 0; i < m1.size(); i++) {
			const auto& currentNode = m1[i]->getNode();
			if (nodes.count(currentNode) == 0) {
				pos1.push_back(std::vector<Eigen::Matrix4f>());
				pos2.push_back(std::vector<Eigen::Matrix4f>());
				nodes[currentNode] = nodes.size() - 1;
			}
			int index = nodes[currentNode];
			pos1[index].push_back(utility::convention::unserialiseTo4x4f(m1[i]->getData()));
			pos2[index].push_back(utility::convention::unserialiseTo4x4f(m2[i]->getData()));
		}
		CalibrationResult result;
		result.systems = SystemPair(m1.front()->getSystem(), m2.front()->getSystem());
		//Compute transform and error
		result.transform = utility::calibration::Transform::twoSystems(pos1, pos2, &result.error);
		//TODO: compute quality
		result.quality = utility::calibration::qualityFromError(result.error, 1);
		result.state = CalibrationResult::State::REFINING;
		return result;
	}

	/////////////////////////////////////////////////////////////////////////////////////////////////////////
	//									Calibrator:Public
	/////////////////////////////////////////////////////////////////////////////////////////////////////////
	void Calibrator::addMeasurement(const Measurement::Ptr& m, const NodeDescriptor& node) {
		calibrationSet.addMeasurement(m, m->getSystem(), node);
	}

	void Calibrator::addMeasurementGroup(const std::vector<std::pair<Measurement::Ptr, NodeDescriptor>>& measurementQueue) {
		//Check there is data corresponding to more than one system for a given node, otherwise useless
		auto measurements = filterLonelyData(measurementQueue);

		//Decide if data is useful
		bool dataNovel = checkChanges(measurements);

		if (dataNovel) {
			//Store the (refs to) the relevant measurements
			//FUSION_LOG("Adding calibration measurments!!");
			for (auto& m : measurements) {
				addMeasurement(m.first, m.second);
			}

		}
	}

	void Calibrator::calibrate()
	{
		//For each unordered pairing of systems, check if there are common nodes
		for (std::set<SystemDescriptor>::iterator system1 = calibrationSet.systems.begin(); system1 != calibrationSet.systems.end(); system1++) {
			for (std::set<SystemDescriptor>::iterator system2 = std::next(system1); system2 != calibrationSet.systems.end(); system2++) {
				CalibrationResult r = getResultsFor(*system1, *system2);
				switch (r.state) {
				case (CalibrationResult::State::UNCALIBRATED):
					//Called multiple times until calibration ready - then it performs calibration and sets r.calibrated
					calibrateInitial(*system1, *system2);
					break;
				case (CalibrationResult::State::REFINING):
					refineCalibration(*system1, *system2);
					break;
				case (CalibrationResult::State::CALIBRATED):
					detectFaults(*system1, *system2);
					break;
				}
			}
		}
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
		calibrationResults[forward] = CalibrationResult();
		calibrationResults[forward].systems = forward;
		return calibrationResults[forward];
	}

}
