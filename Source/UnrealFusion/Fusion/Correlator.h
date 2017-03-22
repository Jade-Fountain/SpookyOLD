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
	private:
		class Data {
			struct Streams{
				std::map<Sensor::Ptr, std::vector<Measurement::Ptr>> sensors;
			}
		private:
			//Contains measurements for the ambiguous sensors
			Streams ambiguous_measurements;
			//Maps ambiguous sensor to set of unambiguous measurements from sensors 
			// attached to nodes which the ambiguous sensor could be located
			std::map<NodeDescriptor, Streams> unambiguous_measurements;

			//Nodes for which data is needed
			std::set<NodeDescriptor> relevant_nodes;

		public:
			//Adds ambiguous measurement to data list
			void addAmbiguous(const Sensor::Ptr& sensor, const Measurement::Ptr& m);
			//Adds unambiguous measurement to data list
			void addUnambiguous(const Sensor::Ptr& sensor, const Measurement::Ptr& m);
			//Check if measurements from sensor s are useful for resolving ambiguities
			bool unambiguousMeasurementNeeded(const Sensor::Ptr& s);
			
		};
		Data data;

	public:
		//Adds measurment for a sensor which is attached to an unknown node
		void addAmbiguousMeasurement(const Measurement::Ptr& m);

		//Adds measurement for a sensor attached to a known node
		void addUnambiguousMeasurementIfNeeded(const Measurement::Ptr& m);

		//Performes identification procedure if possible
		void identify();


	};

}

