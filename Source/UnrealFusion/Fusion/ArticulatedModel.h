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
#include "Articulation.h"
#include "FusionTypes.h"
#include "Utilities/DataStructures.h"

namespace fusion {

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
		//Constant internal structure of the node
		std::vector<Articulation> articulations;

		//Homepose is the default node pose (when theta = 0)
		//If the articulations are not twists, then the home pose is the identity
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
			Transform3D pose = (parent != NULL) ? (parent->getPose()) : (Transform3D::Identity());
			for(int i = 0; i < articulations.size(); i++){
				pose = pose * articulations[i].getTransform(local_state.expectation.col(i));
			}
			return pose;
		}
	public:
		Transform3D getFinalPose(){
			Transform3D pose = parent->getPose();
			for(int i = 0; i < articulations.size(); i++){
				pose = pose * articulations[i].getTransform(local_state.expectation.col(i));
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
			void addNode(const NodeDescriptor & node, const NodeDescriptor & parent, const std::vector<Articulation>& model);

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
}
