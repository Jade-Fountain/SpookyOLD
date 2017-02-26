// Fill out your copyright notice in the Description page of Project Settings.
#include "UnrealFusion.h"
#include "FusionGraph.h"

namespace fusion {

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

	}




}

