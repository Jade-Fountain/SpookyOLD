#include "UnrealFusion.h"
#include "Core.h"
namespace fusion {

	//Adds a new measurement to the system
	void Core::addMeasurement(const Measurement::Ptr& m, const NodeDescriptor& node) {
		//systems[m->system].addMeasurement(m);
		skeleton.addMeasurement(node, m);
	}

	//Computes data added since last fuse() call. Should be called repeatedly	
	void Core::fuse() {
		//Add new data to calibration, with checking for usefulness
		calibrator.addMeasurementGroup(skeleton.getMeasurements());

	}

}
