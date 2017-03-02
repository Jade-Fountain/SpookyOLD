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
#include "FusionGraph.h"
#include "Fusion/Utilities/DataStructures.h"
namespace fusion {


	//Centre of a fusion plant
	class Core{
	private:
		//TODO: (IF NEEDED) Raw data ordered by sytem
		//std::map<SystemDescriptor, SensorSystem> systems;

		//Calibration data per system pair (A,B) = std::pair<SystemDescriptor,SystemDescriptor>
		//sensorTransforms[(A,B)]: A -> B
		Calibrator calibrator;

		//Fused data
		FusionGraph skeleton;

		//Sensor list
		std::map<SystemDescriptor, std::map<SensorID, Sensor::Ptr>> sensors;
		
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

		//Returns mapping from s1 to s2
		CalibrationResult getCalibrationResult(SystemDescriptor s1, SystemDescriptor s2);

		void setMeasurementSensorInfo(Measurement::Ptr& m, SystemDescriptor system, SensorID id);

	};

}