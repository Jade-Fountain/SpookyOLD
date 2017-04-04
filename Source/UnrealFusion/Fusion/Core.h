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
#pragma once

#include <string>
#include <map>
#include "Eigen/Core"
#include "FusionTypes.h"
#include "Calibration.h"
#include "Correlator.h"
#include "ArticulatedModel.h"
#include "Fusion/Utilities/DataStructures.h"
#include "Utilities/TimeProfiling.h"

namespace fusion {


	//Centre of a fusion plant
	class Core{
	private:
		//Measurement buffer for this frame
		std::vector<Measurement::Ptr> measurement_buffer;

		//Class responsible for distinguishing ambiguous sensors
		Correlator correlator;

		//Calibration data per system pair (A,B) = std::pair<SystemDescriptor,SystemDescriptor>
		//sensorTransforms[(A,B)]: A -> B
		Calibrator calibrator;

		//Fused data
		ArticulatedModel skeleton;

		//Sensor list
		std::map<SystemDescriptor, std::map<SensorID, Sensor::Ptr>> sensors;

		//Profiling
		utility::Profiler profiler;
		
	public:

		////////////////////////////////////////////////////
		//					Initialisation
		////////////////////////////////////////////////////

		//Adds a node to the fusion graph model
		void addNode(const NodeDescriptor& node, const NodeDescriptor& parent, const Eigen::Vector3f& boneVec);
		
		//Computes necessary metadata after setup
		void finaliseSetup();

		////////////////////////////////////////////////////
		//					Input at runtime
		////////////////////////////////////////////////////
		
		//Adds a new measurement to the system (unambiguous)
		void addMeasurement(const Measurement::Ptr& m, const NodeDescriptor& node);
		
		//Adds a new measurement to the system (ambiguous or unambiguous, less efficient)
		void addMeasurement(const Measurement::Ptr & m, const std::vector<NodeDescriptor>& nodes);
		
		//Computes data added since last fuse() call. Should be called repeatedly	
		void fuse();

		////////////////////////////////////////////////////
		//					Results
		////////////////////////////////////////////////////

		//Returns the global pose of node
		Transform3D getNodeGlobalPose(const NodeDescriptor& node);
		
		//Returns the local orientation of a node
		Transform3D getNodeLocalPose(const NodeDescriptor& node);

		//Returns mapping from s1 to s2
		CalibrationResult getCalibrationResult(SystemDescriptor s1, SystemDescriptor s2);

		//Returns most likely node location of the given sensor
		NodeDescriptor getCorrelationResult(SystemDescriptor system, SensorID id);

		//Called by owner of the Core object to set a measurement sensor pointer
		void setMeasurementSensorInfo(Measurement::Ptr& m, SystemDescriptor system, SensorID id);

	};

}