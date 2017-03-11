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
#include "UnrealFusion.h"
#include "Calibration.h"
#include "Fusion/Utilities/CalibrationUtilities.h"
#include "Logging.h"
#include "Fusion/Utilities/Conventions.h"

namespace fusion {
	//TODO: refactor this mess
	CalibrationResult Calibrator::calPos(const std::vector<Measurement::Ptr>& m1, const std::vector<Measurement::Ptr>& m2, const CalibrationResult& currentCalibration)	const
	{
		float quality_threshold = 0.8;
		float qualityScaleFactor = 0.05;
		float fault_hysteresis_rate = 1;
		float fault_threshold = 0.75;
		std::vector<Eigen::Vector3f> pos1(m1.size());
		std::vector<Eigen::Vector3f> pos2(m2.size());
		for (int i = 0; i < m1.size(); i++) {
			pos1[i] = m1[i]->getData();
			pos2[i] = m2[i]->getData();
		}
		CalibrationResult result = currentCalibration;
		result.systems = SystemPair(m1.front()->getSystem(), m2.front()->getSystem());
		//Compute transform and error
		switch (currentCalibration.state) {
		case (CalibrationResult::State::UNCALIBRATED):
			result.transform = utility::calibration::Position::calibrateIdenticalPair(pos1, pos2, &result.error);
			result.quality = utility::calibration::qualityFromError(result.error, qualityScaleFactor);
			result.relevance = result.quality;
			FUSION_LOG("CALIBRATED!!! error: " + std::to_string(result.error) + ", quality = " + std::to_string(result.quality));
			result.state = CalibrationResult::State::REFINING;
			break;
		case (CalibrationResult::State::REFINING):
			//refinement calibration
			result.transform = utility::calibration::Position::refineIdenticalPairPosition(pos1, pos2, currentCalibration.transform, &result.error);
			result.quality = utility::calibration::qualityFromError(result.error, qualityScaleFactor);
			if (result.quality > quality_threshold) {
				FUSION_LOG("REFINEMENT FINISHED!!! error: " + std::to_string(result.error) + ", quality = " + std::to_string(result.quality));
				result.state = CalibrationResult::State::CALIBRATED;
			} else {
				FUSION_LOG("REFINING: result.error " + std::to_string(result.error) + ", quality = " + std::to_string(result.quality));
				result.state = CalibrationResult::State::REFINING;
			}
			break;
		case (CalibrationResult::State::CALIBRATED):
			//TODO: distinguish noise vs. actual movement
			//Track new transform and see how much it moves? (expensive)
			float error = utility::calibration::Position::getError(pos1,pos2,currentCalibration.transform);
			result.relevance = result.relevance * (1-fault_hysteresis_rate) + fault_hysteresis_rate * utility::calibration::qualityFromError(error, qualityScaleFactor);
			FUSION_LOG(" Already calibrated - watching for faults - relevance = " + std::to_string(result.relevance) + "vs. quality = " + std::to_string(result.quality));
			//Relevance is the latest quality value, filtered with exponential filter
			//If our quality varies from the expected too much, we need to recalibrate
			if(result.relevance/result.quality < fault_threshold){
				result.state = CalibrationResult::State::UNCALIBRATED;
			} else {
				CalibrationResult::State::CALIBRATED;
			}
			return result;
			break;
		}
		return result;
	}

	CalibrationResult Calibrator::cal6DoF(const std::vector<Measurement::Ptr>& m1, const std::vector<Measurement::Ptr>& m2)const 
	{
		//At least one node
		std::vector<std::vector<Eigen::Matrix4f>> pos1(1);
		std::vector<std::vector<Eigen::Matrix4f>> pos2(1);
		std::map<NodeDescriptor, int> nodes;
		nodes[m1.front()->getNode()] = 0;
		for (int i = 0; i < m1.size(); i++) {
			const auto& currentNode = m1[i]->getNode();
			if (nodes.count(currentNode) == 0) {
				pos1.push_back(std::vector<Eigen::Matrix4f>());
				pos2.push_back(std::vector<Eigen::Matrix4f>());
				nodes[currentNode] = nodes.size() - 1;
			}
			int index = nodes[currentNode];
			pos1[index].push_back(utility::convention::unserialiseTo4x4f(m1[i]->getData()));
			pos2[index].push_back(utility::convention::unserialiseTo4x4f(m2[i]->getData()));
		}
		CalibrationResult result;
		result.systems = SystemPair(m1.front()->getSystem(), m2.front()->getSystem());
		//Compute transform and error
		result.transform = utility::calibration::Transform::twoSystems(pos1, pos2, &result.error);
		//TODO: compute quality
		result.quality = utility::calibration::qualityFromError(result.error, 1);
		result.state = CalibrationResult::State::REFINING;
		return result;
	}
}