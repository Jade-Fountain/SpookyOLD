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
		//Check if cached
		//If we need to recache, concatenate articulations
		if(rechacheRequired){
			//If root node, return identity
			Transform3D pose = (parent != NULL) ? (parent->getPose()) : (Transform3D::Identity());
			for (int i = 0; i < articulations.size(); i++) {
				pose = pose * articulations[i].getTransform(local_state.expectation.col(i));
			}	
			//Save cache
			cachedPose = pose;
			rechacheRequired = false;
		}
		return cachedPose;
	}
	
	//-------------------------------------------------------------------------------------------------------
	//									Public
	//-------------------------------------------------------------------------------------------------------

	Transform3D Node::getFinalPose(){
		Transform3D pose = getPose();
		return pose * homePose;
	}

	void Node::updateState(const State& new_state){
		rechacheRequired = true;
		local_state = new_state;
	}

	void Node::setModel(std::vector<Articulation> art){
		articulations = art;
		std::vector<Eigen::VectorXf> state;
		int max_n_rows = 1;
		for(int i = 0; i < articulations.size(); i++){	
			state.push_back(Articulation::getInitialState(articulations[i].getType()));
			max_n_rows = (max_n_rows < state.back().rows()) ? state.back().rows() : max_n_rows;
		}
		local_state.expectation = Eigen::MatrixXf::Zero(max_n_rows, state.size());
		for (int i = 0; i < articulations.size(); i++) {
			local_state.expectation.col(i) = state[i];
		}
		local_state.variance = initial_covariance * Eigen::MatrixXf::Identity(max_n_rows*state.size(), max_n_rows*state.size());
	}
	/////////////////////////////////////////////////////////////////////////////////////////////////////////
	//									ArticulatedModel
	/////////////////////////////////////////////////////////////////////////////////////////////////////////

	//-------------------------------------------------------------------------------------------------------
	//									Public
	//-------------------------------------------------------------------------------------------------------

	void ArticulatedModel::addNode(const NodeDescriptor & node, const NodeDescriptor & parent) {
		//This line initialises the node entry if not already initialised
		utility::safeAccess(nodes, node)->desc = node;
		nodes[node]->parent_desc = parent;
	}
	
	void ArticulatedModel::enumerateHeirarchy(){
		for(auto& node : nodes){
			NodeDescriptor parent = node.second->parent_desc;
			if(nodes.count(parent) != 0 && parent != node.second->desc){
				node.second->parent = nodes[parent];
			}
		}
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

	void ArticulatedModel::setBoneForNode(const NodeDescriptor& node, const Eigen::Vector3f& boneVec) {
		std::vector<Articulation> art;
		art.push_back(Articulation::createBone(boneVec));
		nodes[node]->setModel(art);
	}

	Transform3D ArticulatedModel::getNodePose(const NodeDescriptor& node){
		if(nodes.count(node) == 0){
			return Transform3D::Identity();
		} else {
			return nodes[node]->getFinalPose();
		}
	}
	//-------------------------------------------------------------------------------------------------------
	//									Private
	//-------------------------------------------------------------------------------------------------------

	void  ArticulatedModel::clearMeasurements() {
		for (auto& node : nodes) {
			node.second->measurements.clear();
		}
	}




}

