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
	//TODO: check novelty
	void Calibrator::addMeasurement(const Measurement::Ptr& m, const NodeDescriptor& node) {
		calibrationSet.addMeasurement(m, m->system, node);
	}
}
