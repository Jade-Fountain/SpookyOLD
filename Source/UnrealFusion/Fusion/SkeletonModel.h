// Fill out your copyright notice in the Description page of Project Settings.

#pragma once
// Fill out your copyright notice in the Description page of Project Settings.
#include <string>
#include <map>
#include "Eigen/Core"
#include "Fusion/FusionTypes.h"

namespace fusion {

	class SkeletonModel{
	private:
		//SkeletonData
		std::vector<DefaultSensorNode> nodes;


	public:
		SkeletonModel();
		~SkeletonModel();
	};

}