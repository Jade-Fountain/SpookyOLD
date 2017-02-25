// Fill out your copyright notice in the Description page of Project Settings.

#pragma once
// Fill out your copyright notice in the Description page of Project Settings.
#include "Fusion/Calibration.h"
#include <string>
#include <map>
#include "Eigen/Core"
#include "Fusion/FusionTypes.h"

class SensorSystem {

};

class SkeletonModel{
private:
	std::vector<DefaultSensorNode> nodes;

	//sensorTransforms[(A,B)]: A -> B
	std::map<std::pair<std::string,std::string>,Calibration> calibrations;
public:
	SkeletonModel();
	~SkeletonModel();
};
