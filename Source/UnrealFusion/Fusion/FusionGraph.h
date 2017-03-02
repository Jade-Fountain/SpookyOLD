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
#include "Utilities/DataStructures.h"

namespace fusion {
	

	//NOTE: templating means each skeleton is of a particular model type
	//TODO: make folder for model.h files
	class CartesianModel{
	public:
		//Data model for the state of the data
		struct State{
			Eigen::Vector3f expectation;
			Eigen::Matrix<float,3,3> uncertainty;
		};
		
		static void updateState(State* state, const Measurement& measurement){
			//Use latest measurement
			state->expectation = measurement.getData().segment(0,3);//start,count
			state->uncertainty = measurement.getUncertainty().topLeftCorner(3,3);
		}
	};

	class FusionGraph {

	/*//////////////////////////////////////////////////////////////////
	* Sensor Nodes model the state of the system with a tree of nodes
	*//////////////////////////////////////////////////////////////////
		template <typename Model>
		struct Node
		{
		public:
			//Current best state estimate, typically including some estimate of variance or confidence
			typename Model::State state;
			//Queued messages, 
			//TODO: order by timestamp
			std::vector<Measurement::Ptr> measurements;

			//Own name
			NodeDescriptor desc;
			//Parent of this node
			NodeDescriptor parent_desc;
			//TODO: Compute children of this node from parents
			//std::vector<NodeDescriptor> children_desc;


			////Called by parent node
			//void update(){
			//	updateState();
			//	for(auto& child : children){
			//		child.update();
			//	}
			//}
			////Computes update according to measurements
			//void updateState(){
			//	while(!measurements.empty()) {
			//		auto& measurement = measurements.pop_front();
			//		Model::updateState(&state, &uncertainty, measurement);
			//	}
			//}
		};
		typedef Node<CartesianModel> DefaultSensorNode;

	/*//////////////////////////////////////////////////////////////////
	*				Public methods
	*//////////////////////////////////////////////////////////////////
	public:
		//Adds node to the skeleton
		void addNode(const NodeDescriptor & node, const NodeDescriptor & parent);

		//Returns a list of pending measurments
		std::vector<std::pair<Measurement::Ptr, NodeDescriptor>> getMeasurements();

		//Adds a measurement to be fused on next fusion call
		void addMeasurement(const NodeDescriptor& node, const Measurement::Ptr& m);

		//Compute best model for given data and prior
		void fuse();
	
	/*//////////////////////////////////////////////////////////////////
	*				Private Data
	*//////////////////////////////////////////////////////////////////
	private:
		//SkeletonData
		std::map<NodeDescriptor, DefaultSensorNode> nodes;
		
		//Clears measurements in graph
		void clearMeasurements();


	};

}