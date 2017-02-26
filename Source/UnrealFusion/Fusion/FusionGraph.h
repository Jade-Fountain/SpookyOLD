// Fill out your copyright notice in the Description page of Project Settings.

#pragma once
// Fill out your copyright notice in the Description page of Project Settings.
#include <string>
#include <map>
#include "Eigen/Core"
#include "FusionTypes.h"

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

		/**
		 * Sensor Nodes model the state of the system with a tree of nodes
		 */
		template <typename Model>
		class Node
		{
		private:
			//Current best state estimate, typically including some estimate of variance or confidence
			typename Model::State state;
			//Queued messages, 
			//TODO: order by timestamp
			std::queue<Measurement::Ptr> measurements;
			//Children of this node
			std::vector<int> children_indices;
			//Parent of this node
			int parent_index;
		public:
			//Called by parent node
			void update(){
				updateState();
				for(auto& child : children){
					child.update();
				}
			}
			//Computes update according to measurements
			void updateState(){
				while(!measurements.empty()) {
					auto& measurement = measurements.pop_front();
					Model::updateState(&state, &uncertainty, measurement);
				}
			}
		};
		typedef Node<CartesianModel> DefaultSensorNode;


	private:
		//SkeletonData
		std::vector<DefaultSensorNode> nodes;


	};

}