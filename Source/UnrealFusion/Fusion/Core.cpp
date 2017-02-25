#include "UnrealFusion.h"
#include "Core.h"
namespace fusion {

	//Adds a new measurement to the system
	void Core::addMeasurement(Measurement::Ptr m, std::string node_name) {
		systems[m->system].addMeasurement(m);
		skeleton->addMeasurement(node_name, m);
	}

	//Computes data added since last fuse() call. Should be called repeatedly	
	void Core::fuse() {
		
	}

}
