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
		private:
			std::set<Sensor::Ptr> ambiguous_sensors;
			std::map<Sensor::Ptr, std::vector<Measurement::Ptr>> ambiguous_measurements;
			std::map<Sensor::Ptr, std::vector<Measurement::Ptr>> unambiguous_measurements;

		public:
			
			void addAmbiguous(const Sensor::Ptr& sensor, const Measurement::Ptr& m) {
				if (ambiguous_measurements.count(sensor) == 0) {
					ambiguous_measurements[sensor] = std::vector<Measurement::Ptr>();
					ambiguous_sensors.insert(sensor);
				} else {
					ambiguous_measurements[sensor].push_back(m);
				}
			}

			void addUnambiguous(const Sensor::Ptr& sensor, const Measurement::Ptr& m) {
				utility::safeAccess(unambiguous_measurements, sensor).push_back(m);
			}

			bool unambiguousMeasurementNeeded(const Sensor::Ptr& s) {
				return ambiguous_sensors.count(s) >= 1;
			}
			
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

