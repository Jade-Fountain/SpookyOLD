// Fill out your copyright notice in the Description page of Project Settings.
#include "UnrealFusion.h"
#include "FusionGraph.h"

namespace fusion {
	
	void FusionGraph::addNode(const NodeDescriptor & node, const NodeDescriptor & parent) {
		utility::safeAccess(nodes, node).desc = node;
		utility::safeAccess(nodes, node).parent_desc = parent;
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

