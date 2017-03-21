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
#include "Fusion/Utilities/DataStructures.h"
namespace fusion {


	//Centre of a fusion plant
	class Correlator {
	public:
		//Adds measurment for a sensor which is attached to an unknown node
		void addAmbiguousMeasurement(Measurement::Ptr m);

		//Adds measurement for a sensor attached to a known node
		void addUnambiguousMeasurementIfNeeded(Measurement::Ptr m);

		//Performes identification procedure if possible
		void identify();
	};

}

