#pragma once

#include <string>
#include <map>
#include "Eigen/Core"
#include "FusionTypes.h"
#include "Calibration.h"
#include "FusionGraph.h"

namespace fusion {

	class SensorSystem {

	};

	//Centre of a fusion plant
	class Core{
	private:
		//Raw data ordered by sytem
		std::map<SystemDescriptor, SensorSystem> systems;

		//Calibration data per system pair (A,B) = std::pair<SystemDescriptor,SystemDescriptor>
		//sensorTransforms[(A,B)]: A -> B
		Calibrator calibrator;

		//Fused data
		FusionGraph skeleton;

		//TODO?:Queue of latest measurements
		
	public:

		////////////////////////////////////////////////////
		//					Initialisation
		////////////////////////////////////////////////////

		//Adds a node to the fusion graph model
		void addNode(const NodeDescriptor& node, const NodeDescriptor& parent);
		
		//Computes necessary metadata after setup
		void finaliseSetup();

		////////////////////////////////////////////////////
		//					Runtime
		////////////////////////////////////////////////////
		//Adds a new measurement to the system
		void addMeasurement(const Measurement::Ptr& m, const NodeDescriptor& node);
		
		//Computes data added since last fuse() call. Should be called repeatedly	
		void fuse();


	};

}