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
#include "UnrealFusion.h"
#include "FusionGraph.h"

namespace fusion {
	
	void FusionGraph::addNode(const NodeDescriptor & node, const NodeDescriptor & parent) {
		utility::safeAccess(nodes, node)->desc = node;
		utility::safeAccess(nodes, node)->parent_desc = parent;
	}


	std::vector<std::pair<Measurement::Ptr, NodeDescriptor>> FusionGraph::getMeasurements() {
		std::vector<std::pair<Measurement::Ptr, NodeDescriptor>> measurements;
		for (auto& node : nodes) {
			for (auto& measurement : node.second.measurements) {
				measurements.push_back(std::make_pair(measurement,node.first));
			}
		}
		return measurements;
	}

	void FusionGraph::addMeasurement(const NodeDescriptor& node, const Measurement::Ptr& m) {
		utility::safeAccess(nodes,node).measurements.push_back(m);
	}

	void FusionGraph::fuse() {
		//TODO: implement fusion
		//For now, just empty measurements
		clearMeasurements();
	}

	void  FusionGraph::clearMeasurements() {
		for (auto& node : nodes) {
			node.second.measurements.clear();
		}
	}




}

