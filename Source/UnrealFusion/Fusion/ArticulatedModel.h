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
#include "Eigen/Core"
#include "FusionTypes.h"
#include "Utilities/DataStructures.h"

class Articulation{
public:
	//Axis or axis + angle
	Eigen::Vector3f w;
	//Vector: twist offest or displacement 
	Eigen::Vector3f v;

	//Type of articulation
	enum Type{
		CARTESIAN = 0,
		TWIST = 1,
		BONE = 2
	}

	Transform3D getTransform(Eigen::VectorXf theta){
		switch(type){
			//TODO: actually implement these things / use library
			case(CARTESIAN):
				return translation(v) * exp(theta(0) * skew(w.normalise()));
				break;
			case(TWIST):
				Eigen::Matrix4f zeta = Eigen::Matrix4f::Identity();
				zeta.topLeftCorner(3,3) = skew(w);
				zeta.column(3) = v;
				zeta(3,3) = 0;
				return exp(theta(0) * zeta);
				break;
			case(BONE):
				//v should be along the x-axis for this one
				return translation(v) * exp(skew(theta));
				break;

		}		
	}

	
};

class Node{
public:
	//Define ptr type for neater code
	typedef std::shared_ptr<Node> Ptr;

	//////////////////////////////////////////////////////////////////
	//Internal Info
	//////////////////////////////////////////////////////////////////
	struct State{
		//Vectors of articulation states stored in columns
		Eigen::MatrixXf expectation;
		//Covariance associated with vec(expectation)
		Eigen::MatrixXf uncertainty;
	};
	
	//Current state
	State local_state;
	//Constant structure
	std::vector<Articulation> structure;

	//Homepose is the default node pose (when theta = 0)
	//If the articulations are 
	Transform3D homePose;

	//Pending measurements 
	//TODO: ensure ordered by timestamp
	std::vector<Measurement::Ptr> measurements;
	
	//////////////////////////////////////////////////////////////////
	//External info
	//////////////////////////////////////////////////////////////////

	//Own name
	NodeDescriptor desc;

	//Parent of this node
	Ptr parent;
	NodeDescriptor parent_desc;

private:
	Transform3D getPose(){
		Transform3D pose = (parent != NULL) ? (parent->getPose()) : (Eigen::Matrix4f::Identity());
		for(int i = 0; i < structure.size(); i++){
			pose = pose * structure[i].getTransform(state.expectation.col(i));
		}
		return pose;
	}
public:
	Transform3D getFinalPose(){
		Transform3D pose = parent->getPose();
		for(int i = 0; i < structure.size(); i++){
			pose = pose * structure[i].getTransform(state.expectation.col(i));
		}
		return pose * homePose;
	}

	void updateState(const State& new_state){
		local_state = new_state;
	}
	
};

class ArticulatedModel{



	/*//////////////////////////////////////////////////////////////////
	*				Public methods
	*//////////////////////////////////////////////////////////////////
	public:
		//Adds node to the skeleton
		void addNode(const NodeDescriptor & node, const NodeDescriptor & parent);

		//Returns a list of pending measurements
		std::vector<std::pair<Measurement::Ptr, NodeDescriptor>> getMeasurements();

		//Adds a measurement to be fused on next fusion call
		void addMeasurement(const NodeDescriptor& node, const Measurement::Ptr& m);

		//Compute best model for given data and prior
		void fuse();


		//WorldState getWorldState(NodeDescriptor node) {
		//	//TODO: check for cached state
		//	if (nodes.count(node) == 0) {
		//		return WorldState();
		//	}
		//	const auto& n = nodes[node];
		//	return TwistModel::worldStateFunc(n.state,getWorldState(n.parent_desc))
		//}

	
	/*//////////////////////////////////////////////////////////////////
	*				Private Data
	*//////////////////////////////////////////////////////////////////
	private:
		//SkeletonData
		std::map<NodeDescriptor,Node::Ptr> nodes;
		
		//Clears measurements in graph
		void clearMeasurements();

};