#include "UnrealFusion.h"
#include "Calibration.h"

namespace fusion {
	/////////////////////////////////////////////////////////////////////////////////////////////////////////
	//									CalibrationDataSet
	/////////////////////////////////////////////////////////////////////////////////////////////////////////

	void CalibrationDataSet::addMeasurement(const Measurement::Ptr& m, const SystemDescriptor& system, const NodeDescriptor& node) {
		SystemNodePair key = SystemNodePair(system, node);
		data[key].addMeasurement(m);
	}

	void CalibrationDataSet::Stream::addMeasurement(const Measurement::Ptr& m) {
		sensors[m->sensorID].push_back(m);
	}




	/////////////////////////////////////////////////////////////////////////////////////////////////////////
	//									Calibrator
	/////////////////////////////////////////////////////////////////////////////////////////////////////////

	std::vector<std::pair<Measurement::Ptr, NodeDescriptor>>
		Calibrator::filterLonelyData(const std::vector<std::pair<Measurement::Ptr, NodeDescriptor>>& measurementQueue) {
		std::vector<std::pair<Measurement::Ptr, NodeDescriptor>> result;
		SafeMap<NodeDescriptor, std::set<SystemDescriptor>> systemsPerNode;
		for (auto& m : measurementQueue) {
			systemsPerNode[m.second].insert(m.first->system);
		}
		for (auto& m : measurementQueue) {
			if (systemsPerNode[m.second].size() > 1) {
				result.push_back(m);
			}
		}
		return result;
	}

	void Calibrator::addMeasurement(const Measurement::Ptr& m, const NodeDescriptor& node) {
		calibrationSet.addMeasurement(m, m->system, node);
	}

	void Calibrator::addMeasurementGroup(const std::vector<std::pair<Measurement::Ptr, NodeDescriptor>>& measurementQueue) {
		//TODO: Decide if data is useful
		
		//Check there is data corresponding to more than one system for a given node, otherwise useless
		auto measurements = filterLonelyData(measurementQueue);

		//Store the (refs to) the relevant measurements
		for (auto& m : measurements) {
			addMeasurement(m.first, m.second);
		}
	}

}
