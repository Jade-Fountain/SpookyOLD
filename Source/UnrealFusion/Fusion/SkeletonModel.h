// Fill out your copyright notice in the Description page of Project Settings.

#pragma once
// Fill out your copyright notice in the Description page of Project Settings.

#include <string>
#include <map>
#include "Eigen/Core"
#include "Fusion/FusionTypes.h"

struct CalibrationData{
	struct Data {
		
	};
	bool calibrated = false;
	Eigen::Matrix4f transform;
	std::vector<Data> dataSamples;
};

class SkeletonModel{
private:
	std::vector<DefaultSensorNode> nodes;

	//sensorTransforms[(A,B)]: A -> B
	std::map<std::pair<std::string,std::string>,CalibrationData> sensorTransforms;
public:
	SkeletonModel();
	~SkeletonModel();
};
