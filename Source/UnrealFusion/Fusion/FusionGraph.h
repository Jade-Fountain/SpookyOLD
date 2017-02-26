// Fill out your copyright notice in the Description page of Project Settings.

#pragma once
// Fill out your copyright notice in the Description page of Project Settings.
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
	
	/*//////////////////////////////////////////////////////////////////
	*				Private Data
	*//////////////////////////////////////////////////////////////////
	private:
		//SkeletonData
		std::map<NodeDescriptor, DefaultSensorNode> nodes;



	};

}