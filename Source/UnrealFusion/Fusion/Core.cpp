#include "UnrealFusion.h"
#include "Core.h"
namespace fusion {

	//Adds a new measurement to the system
	void Core::addMeasurement(const Measurement::Ptr& m, const NodeDescriptor& node) {
		//systems[m->system].addMeasurement(m);
		//skeleton->addMeasurement(node, m);
		calibrator.addMeasurement(m, node);
	}

	//Computes data added since last fuse() call. Should be called repeatedly	
	void Core::fuse() {
//		if()
	}

}
