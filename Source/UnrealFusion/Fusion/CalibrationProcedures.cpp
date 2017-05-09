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
		float quality_convergence_threshold = 0.1;
		float qualityScaleFactor = 0.05;
		float fault_hysteresis_rate = 1;
		float settle_threshold = 0.85;
		float fault_threshold = 0.90;
		std::vector<Eigen::Vector3f> pos1(m1.size());
		std::vector<Eigen::Vector3f> pos2(m2.size());
		std::vector<Eigen::Matrix3f> inverse_variances(m1.size());
		for (int i = 0; i < m1.size(); i++) {
			pos1[i] = m1[i]->getPosition();
			pos2[i] = m2[i]->getPosition();
			//TODO: Not strictly correct
			inverse_variances[i] = (m1[i]->getPositionVar() + m2[i]->getPositionVar()).inverse();
		}
		CalibrationResult result = currentCalibration;
		result.systems = SystemPair(m1.front()->getSystem(), m2.front()->getSystem());
		//Compute transform and error
		switch (currentCalibration.state) {
			case (CalibrationResult::State::UNCALIBRATED):
			{
				//Clear previous
				result.transform.setIdentity();
				//result.transform = utility::calibration::Position::calibrateWeightedIdenticalPair(pos1, pos2, inverse_variances, &result.error);
				result.transform = utility::calibration::Position::calibrateIdenticalPairTransform(pos1, pos2, &result.error);
				for (int i = 0; i < 10; i++) {
					result.transform = utility::calibration::Position::refineIdenticalPairPosition(pos1, pos2, result.transform, &result.error);
					result.transform = utility::calibration::Position::refineIdenticalPairRotation(pos1, pos2, result.transform, &result.error);
				}
				result.quality = utility::qualityFromError(result.error, qualityScaleFactor);
				result.relevance = result.quality;
				FUSION_LOG("CALIBRATED!!! error: " + std::to_string(result.error) + ", quality = " + std::to_string(result.quality));
				//DEBUG:: Straight to calibrated
				result.state = CalibrationResult::State::REFINING;
				break;
			}
			case (CalibrationResult::State::REFINING):
			{
				//refinement calibration
				result.transform = utility::calibration::Position::refineIdenticalPairPosition(pos1, pos2, result.transform, &result.error);
				result.transform = utility::calibration::Position::refineIdenticalPairRotation(pos1, pos2, result.transform, &result.error);
				float new_quality = utility::qualityFromError(result.error, qualityScaleFactor);
				if (new_quality - result.quality > quality_convergence_threshold) {
					//If large gains are being made keep going
					FUSION_LOG("REFINING: new_quality = " + std::to_string(new_quality) + ", quality = " + std::to_string(result.quality));
					result.state = CalibrationResult::State::REFINING;
				}
				else if (new_quality > settle_threshold) {
					//If change is a small improvement, we are done
					FUSION_LOG("REFINEMENT FINISHED!!! new_quality = " + std::to_string(new_quality) + ", quality = " + std::to_string(result.quality));
					result.state = CalibrationResult::State::CALIBRATED;
				}
				else {
					//If we cant improve, then we probably started in a bad state, so start again
					FUSION_LOG("Starting over: new_quality = " + std::to_string(new_quality) + ", quality = " + std::to_string(result.quality));
					result.state = CalibrationResult::State::UNCALIBRATED;
				}
				result.quality = new_quality;
				break;
			}
			case (CalibrationResult::State::CALIBRATED):
			{
				//TODO: distinguish noise vs. actual movement
				//TODO: implement that fault decay detection thing
				//Track new transform and see how much it moves? (expensive)
				float error = utility::calibration::Position::getError(pos1, pos2, currentCalibration.transform);
				result.relevance = result.relevance * (1 - fault_hysteresis_rate) + fault_hysteresis_rate * utility::qualityFromError(error, qualityScaleFactor);
				FUSION_LOG(" Already calibrated - watching for faults - relevance = " + std::to_string(result.relevance) + " vs. quality = " + std::to_string(result.quality));
				FUSION_LOG(" relevance / quality = " + std::to_string(result.relevance / result.quality) + " vs. thres = " + std::to_string(fault_threshold));
				//Relevance is the latest quality value, filtered with exponential filter
				//If our quality varies from the expected too much, we need to recalibrate
				if (result.relevance / result.quality < fault_threshold) {
					//Try to improve the relevance
					result.state = CalibrationResult::State::REFINING;
					result.quality = result.relevance;
				}
				else {
					result.state = CalibrationResult::State::CALIBRATED;
				}
				return result;
				break;
			}
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
		result.quality = utility::qualityFromError(result.error, 1);
		result.state = CalibrationResult::State::REFINING;
		return result;
	}

	float Calibrator::estimateLatencies(const std::vector<Measurement::Ptr>& meas1, const std::vector<Measurement::Ptr>& meas2) {
		float last_timestamp = meas1.front()->getTimestamp();
		int i = 0;
		int first = i;
		float result_sum = 0;
		int count_streams = 0;
		while (i < meas1.size()) {
			//Should be synchronised at this point
			if(i != first && meas1[i]->getTimestamp() < last_timestamp) {
				std::vector<Measurement::Ptr> m1(meas1.begin() + first, meas1.begin() + i);//Excludes last
				std::vector<Measurement::Ptr> m2(meas2.begin() + first, meas2.begin() + i);
				result_sum += estimateLatency(m1, m2);
				first = i;
				count_streams++;
			}
			last_timestamp = meas1[i]->getTimestamp();
			i++;
		}
		return result_sum / count_streams;

	}

	float Calibrator::estimateLatency(const std::vector<Measurement::Ptr>& m1, const std::vector<Measurement::Ptr>& m2) {
		Eigen::VectorXf data1(m1.size());
		Eigen::VectorXf times1(m1.size());
		for (int i = 0; i < m1.size() - 1; i++) {
			//Velocity data
			data1[i] = m1[i]->compare(m1[0]);
			//data1[i] = m1[i+1]->compare(m1[i]) / (m1[i + 1]->getTimestamp() - m1[i]->getTimestamp());
			//timestamps
			times1[i] = m1[i]->getTimestamp();
		}

		Eigen::VectorXf data2(m2.size());
		Eigen::VectorXf times2(m2.size());
		for (int i = 0; i < m2.size() - 1; i++) {
			//Velocity data
			data2[i] = m2[i]->compare(m2[0]);
			//data2[i] = m2[i+1]->compare(m2[i]) / (m2[i + 1]->getTimestamp() - m2[i]->getTimestamp());
			//timestamps
			times2[i] = m2[i]->getTimestamp();
		}
		//return latency of m2 relative to m1
		return utility::calibration::Time::estimateLatency(data1, times1, data2, times2);

	}

}