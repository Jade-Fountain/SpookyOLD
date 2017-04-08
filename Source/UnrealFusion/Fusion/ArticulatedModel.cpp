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
#include "Utilities/Conventions.h"

namespace fusion {
	/////////////////////////////////////////////////////////////////////////////////////////////////////////
	//									Node
	/////////////////////////////////////////////////////////////////////////////////////////////////////////

	//-------------------------------------------------------------------------------------------------------
	//									Public
	//-------------------------------------------------------------------------------------------------------
	Node::Node() {
		homePose = Transform3D::Identity();
	}

	Transform3D Node::getFinalGlobalPose(){
		Transform3D pose = getGlobalPose();
		return pose * homePose;
	}

	Transform3D Node::getLocalPose(){
		Transform3D pose = Transform3D::Identity();
		for (int i = 0; i < articulations.size(); i++) {
			pose = pose * articulations[i].getTransform(local_state.expectation.col(i));
		}	
		return pose;
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

	void Node::fuse(){
		Transform3D parent_pose = Transform3D::Identity();
		if (parent != NULL) {
			parent->fuse();
			parent_pose = parent->getGlobalPose();
		}
		if(articulations[0].getType() == Articulation::Type::BONE || articulations[0].getType() == Articulation::Type::POSE){
			for(auto& m : measurements){
				if (m->confidence < 0.75) {
					continue;
				}

				if(m->type == Measurement::Type::ROTATION ||
				(articulations[0].getType() == Articulation::Type::BONE && m->type == Measurement::Type::RIGID_BODY) )
				{
					Node::State new_state;
					//Simple update based on parent pose
					new_state.expectation = Eigen::Quaternionf(parent_pose.rotation().inverse() * m->getRotation()).coeffs();
					new_state.expectation.normalize();
					//TODO: make names consitent
					new_state.variance = m->getRotationVar();
					updateState(new_state);
				} 
				else if (m->type == Measurement::Type::RIGID_BODY && articulations[0].getType() == Articulation::Type::POSE)
				{
					Node::State new_state;
					new_state.variance = m->getPosQuatVar();
					//Simple update based on parent pose
					Eigen::Quaternionf qLocal= Eigen::Quaternionf(parent_pose.rotation().inverse() * m->getRotation());
					Eigen::Vector3f posLocal = parent_pose.inverse() * m->getPosition();
					new_state.expectation = Eigen::Matrix<float, 7, 1>::Zero();
					new_state.expectation << posLocal, qLocal.coeffs();
					updateState(new_state);
				}
			}
			//Dont use data twice
			measurements.clear();
		}
	}

	//-------------------------------------------------------------------------------------------------------
	//									Private
	//-------------------------------------------------------------------------------------------------------

	Transform3D Node::getGlobalPose() {
		//Check if cached
		//If we need to recache, concatenate articulations
		if(rechacheRequired){
			//If root node, return identity
			Transform3D parent_pose = (parent != NULL) ? (parent->getGlobalPose()) : (Transform3D::Identity());
			//Save cache
			cachedPose = parent_pose * getLocalPose();
			rechacheRequired = false;
		}
		return cachedPose;
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

	void ArticulatedModel::addMeasurement(const Measurement::Ptr& m) {
		if(m->isResolved()){
			nodes[m->getNode()]->measurements.push_back(m);
		}
	}

	void ArticulatedModel::addMeasurementGroup(const std::vector<Measurement::Ptr>& measurements) {
		for(auto& m : measurements){
			addMeasurement(m);
		}
	}

	void ArticulatedModel::fuse() {
		//TODO: implement fusion
		for(auto& node : nodes){
			//TODO: support other fusion methods
			node.second->fuse();
		}
		//For now, just empty measurements
		clearMeasurements();
	}

	void ArticulatedModel::setBoneForNode(const NodeDescriptor& node, const Transform3D& boneTransform) {
		std::vector<Articulation> art;
		art.push_back(Articulation::createBone(boneTransform.translation()));
		nodes[node]->setModel(art);
		nodes[node]->local_state.expectation = Eigen::Quaternionf(boneTransform.rotation()).coeffs();
	}


	void ArticulatedModel::setPoseNode(const NodeDescriptor& node, const Transform3D& poseTransform) {
		std::vector<Articulation> art;
		art.push_back(Articulation::createPose());
		nodes[node]->setModel(art);
		nodes[node]->local_state.expectation = Measurement::getPosQuatFromTransform(poseTransform);
	}

	Transform3D ArticulatedModel::getNodeGlobalPose(const NodeDescriptor& node){
		if(nodes.count(node) == 0){
			return Transform3D::Identity();
		} else {
			return nodes[node]->getFinalGlobalPose();
		}
	}

	Transform3D ArticulatedModel::getNodeLocalPose(const NodeDescriptor& node){
		if(nodes.count(node) == 0){
			return Transform3D::Identity();
		} else {
			return nodes[node]->getLocalPose();
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

