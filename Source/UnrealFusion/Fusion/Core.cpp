#include "UnrealFusion.h"
#include "Core.h"
namespace fusion {

	//Adds a new measurement to the system
	void Core::addMeasurement(const Measurement::Ptr& m, const NodeDescriptor& node) {
		//systems[m->system].addMeasurement(m);
		//skeleton->addMeasurement(node, m);
	}

	//Computes data added since last fuse() call. Should be called repeatedly	
	void Core::fuse() {
		std::vector<std::pair<Measurement::Ptr,NodeDescriptor>> measurementQueue = skeleton.getMeasurements();
		//TODO: check novelty of measurement frame
		for (auto& m : measurementQueue) {
			calibrator.addMeasurement(m.first,//Measurement
									  m.second);//NodeDescriptor
		}

//		if()
	}

}
