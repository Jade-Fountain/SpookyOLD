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
#include "ArticulatedModel.h"

namespace fusion {
	/////////////////////////////////////////////////////////////////////////////////////////////////////////
	//									Node
	/////////////////////////////////////////////////////////////////////////////////////////////////////////

	//-------------------------------------------------------------------------------------------------------
	//									Private
	//-------------------------------------------------------------------------------------------------------

	Transform3D Node::getPose() {
		Transform3D pose = (parent != NULL) ? (parent->getPose()) : (Transform3D::Identity());
		for (int i = 0; i < articulations.size(); i++) {
			pose = pose * articulations[i].getTransform(local_state.expectation.col(i));
		}
		return pose;
	}
	
	//-------------------------------------------------------------------------------------------------------
	//									Public
	//-------------------------------------------------------------------------------------------------------

	Transform3D Node::getFinalPose(){
		Transform3D pose = parent->getPose();
		for(int i = 0; i < articulations.size(); i++){
			pose = pose * articulations[i].getTransform(local_state.expectation.col(i));
		}
		return pose * homePose;
	}

	void Node::updateState(const State& new_state){
		local_state = new_state;
	}
	/////////////////////////////////////////////////////////////////////////////////////////////////////////
	//									ArticulatedModel
	/////////////////////////////////////////////////////////////////////////////////////////////////////////
	
	void ArticulatedModel::addNode(const NodeDescriptor & node, const NodeDescriptor & parent, const std::vector<Articulation>& model) {
		//This line initialises the node entry if not already initialised
		utility::safeAccess(nodes, node)->desc = node;
		nodes[node]->parent_desc = parent;
		nodes[node]->articulations = model;
	}


	std::vector<std::pair<Measurement::Ptr, NodeDescriptor>> ArticulatedModel::getMeasurements() {
		std::vector<std::pair<Measurement::Ptr, NodeDescriptor>> measurements;
		for (auto& node : nodes) {
			for (auto& measurement : node.second->measurements) {
				measurements.push_back(std::make_pair(measurement,node.first));
			}
		}
		return measurements;
	}

	void ArticulatedModel::addMeasurement(const NodeDescriptor& node, const Measurement::Ptr& m) {
		nodes[node]->measurements.push_back(m);
	}

	void ArticulatedModel::fuse() {
		//TODO: implement fusion
		//For now, just empty measurements
		clearMeasurements();
	}

	void  ArticulatedModel::clearMeasurements() {
		for (auto& node : nodes) {
			node.second->measurements.clear();
		}
	}




}

