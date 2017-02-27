#include "UnrealFusion.h"
#include "Core.h"
namespace fusion {

	void Core::addNode(const NodeDescriptor & node, const NodeDescriptor & parent)
	{
		skeleton.addNode(node, parent);
	}

	void Core::finaliseSetup()
	{
	}

	//Adds a new measurement to the system
	void Core::addMeasurement(const Measurement::Ptr& m, const NodeDescriptor& node) {
		//systems[m->system].addMeasurement(m);
		skeleton.addMeasurement(node, m);
	}

	//Computes data added since last fuse() call. Should be called repeatedly	
	void Core::fuse() {
		//Add new data to calibration, with checking for usefulness
		calibrator.addMeasurementGroup(skeleton.getMeasurements());
		calibrator.calibrate();
		skeleton.fuse();
	}

}
