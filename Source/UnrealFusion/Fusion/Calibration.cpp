#include "UnrealFusion.h"
#include "Calibration.h"

namespace fusion {
	/////////////////////////////////////////////////////////////////////////////////////////////////////////
	//									CalibrationDataSet
	/////////////////////////////////////////////////////////////////////////////////////////////////////////

	void CalibrationDataSet::Stream::addMeasurement(const Measurement::Ptr& m) {
		sensors[m->sensorID].push_back(m);
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
		std::vector<Measurement::Ptr>& stream = systemNodeTable[sysNode].sensors[m->sensorID];
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
		SafeMap<NodeDescriptor, std::set<SystemDescriptor>> systemsPerNode;
		//Count
		for (auto& m : measurementQueue) {
			systemsPerNode[m.second].insert(m.first->system);
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
		for (auto& m : measurements) {
			auto& mes = m.first;
			auto& node = m.second;
			float diff = calibrationSet.compareMeasurement(mes, mes->system, node);
			//TODO:Perform next check over each node individually
			//If any of the measurements are new then return true
			if (diff > threshold) {
				return true;
			}
		}
		return false;
	}
	/////////////////////////////////////////////////////////////////////////////////////////////////////////
	//									Calibrator:Public
	/////////////////////////////////////////////////////////////////////////////////////////////////////////
	void Calibrator::addMeasurement(const Measurement::Ptr& m, const NodeDescriptor& node) {
		calibrationSet.addMeasurement(m, m->system, node);
	}

	void Calibrator::addMeasurementGroup(const std::vector<std::pair<Measurement::Ptr, NodeDescriptor>>& measurementQueue) {
		
		//Check there is data corresponding to more than one system for a given node, otherwise useless
		auto measurements = filterLonelyData(measurementQueue);
		
		//Decide if data is useful
		bool dataNovel = checkChanges(measurements);

		if (dataNovel) {
			//Store the (refs to) the relevant measurements
			for (auto& m : measurements) {
				addMeasurement(m.first, m.second);
			}

		}
	}

	void Calibrator::calibrate()
	{
		std::map < NodeDescriptor, std::vector<SystemDescriptor>> calibrationGroup;
		//For each unordered pairing of systems, check if there are common nodes
		for (std::set<SystemDescriptor>::iterator system1 = calibrationSet.systems.begin(); system1 != calibrationSet.systems.end(); system1++) {
			for (std::set<SystemDescriptor>::iterator system2 = std::next(system1); system2 != calibrationSet.systems.end(); system2++) {
				for (auto& node : calibrationSet.nodes) {
					
					SystemNodePair sysNode1(*system1, node);
					SystemNodePair sysNode2(*system2, node);
					
					//If there is an entry for each system in the table, add the systems to the node calibration group
					if (calibrationSet.systemNodeTable.count(sysNode1) > 0 &&
						calibrationSet.systemNodeTable.count(sysNode2) > 0) {
						//TODO:Check data counts

						safeAccess(calibrationGroup, node).push_back(*system1);
						safeAccess(calibrationGroup, node).push_back(*system2);
					}
				}
			}
		}
	}

}
