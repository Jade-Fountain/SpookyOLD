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
#include "FusionTypes.h"
#include "Fusion/Utilities/CalibrationUtilities.h"
#include "Logging.h"
#include "Fusion/Utilities/Conventions.h"

namespace fusion {
	//TODO: refactor this mess
	CalibrationResult Calibrator::calPos(const std::vector<Measurement::Ptr>& m1, const std::vector<Measurement::Ptr>& m2, const CalibrationResult& currentCalibration)	const
	{
		float initial_quality_threshold = 0.5;
		float quality_convergence_threshold = 0.1;
		float qualityScaleFactor = 0.05;
		float fault_hysteresis_rate = 1;
		float settle_threshold = 0.85;
		float fault_threshold = 0.90;

		//Chunks corresponding to different sensor pairs
		std::vector<int> chunks;
		chunks.push_back(0);
		int last_id_1 = m1.front()->getSensorID();
		int last_id_2 = m2.front()->getSensorID();

		//Data for each stream
		std::vector<Eigen::Vector3f> pos1(m1.size());
		std::vector<Eigen::Vector3f> pos2(m2.size());
		std::vector<Eigen::Matrix3f> inverse_variances(m1.size());
		std::stringstream ss;
		ss << "DATA[" << m1.front()->getSensor()->system.name << ", " << m2.front()->getSensor()->system.name << "]" <<  std::endl;
		for (int i = 0; i < m1.size(); i++) {
			
			pos1[i] = m1[i]->getPosition();
			pos2[i] = m2[i]->getPosition();
			ss << pos1[i].transpose() << " " << pos2[i].transpose() << std::endl;
			//TODO: Not strictly correct
			inverse_variances[i] = (m1[i]->getPositionVar() + m2[i]->getPositionVar()).inverse();
			
			//If one of the sensor ids has changed, start new chunk
			if (last_id_1 != m1[i]->getSensorID() || last_id_2 != m2[i]->getSensorID()) {
				chunks.push_back(i);
				last_id_1 = m1[i]->getSensorID();
				last_id_2 = m2[i]->getSensorID();
			}
		}

		//Last chunk ends here
		chunks.push_back(m1.size());

		ss << "Chunks: " << chunks.size() << std::endl;

		//Build chunked lists for later:
		//TODO: unhack this whole chunk thing
		std::vector<std::vector<Eigen::Vector3f>> chunked_pos1;
		std::vector<std::vector<Eigen::Vector3f>> chunked_pos2;
		std::vector<std::vector<Eigen::Matrix3f>> chunked_inverse_variances;
		for (int i = 0; i < chunks.size()-1; i++) {
			chunked_pos1.push_back(std::vector<Eigen::Vector3f>(pos1.begin() + chunks[i], pos1.begin() + chunks[i + 1]));
			chunked_pos2.push_back(std::vector<Eigen::Vector3f>(pos2.begin() + chunks[i], pos2.begin() + chunks[i + 1]));
			chunked_inverse_variances.push_back(std::vector<Eigen::Matrix3f>(inverse_variances.begin()+chunks[i], inverse_variances.begin()+chunks[i+1]));
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
					//TODO:clean up
					std::vector<Transform3D> transforms;
					std::vector<float> weights;
					for (int j = 0; j < chunks.size() - 1; j++) {
						weights.push_back(100000);
						transforms.push_back(utility::calibration::Position::refineIdenticalPairPosition(chunked_pos1[j], chunked_pos2[j], result.transform, &weights.back()));
						weights.back() = utility::qualityFromError(weights.back(), qualityScaleFactor);
					}
					result.transform = getMeanTransform(transforms,weights);

					transforms.clear();
					weights.clear();
					for (int j = 0; j < chunks.size() - 1; j++) {
						weights.push_back(100000);
						transforms.push_back(utility::calibration::Position::refineIdenticalPairRotation(chunked_pos1[j], chunked_pos2[j], result.transform, &weights.back()));
						weights.back() = utility::qualityFromError(weights.back(), qualityScaleFactor);
					}
					result.transform = getMeanTransform(transforms, weights);

					//TODO:clean up
				}
				//TODO: proper error
				result.quality = utility::qualityFromError(result.error, qualityScaleFactor);
				result.relevance = result.quality;
				FUSION_LOG("CALIBRATED!!! error: " + std::to_string(result.error) + ", quality = " + std::to_string(result.quality));
				//DEBUG:: Straight to calibrated
				if(result.quality > initial_quality_threshold){
					result.state = CalibrationResult::State::CALIBRATED;
					FUSION_LOG("Calibration passed required threshold: " + std::to_string(initial_quality_threshold) + ", quality = " + std::to_string(result.quality));
				}
				else {
					FUSION_LOG("Calibration DID NOT PASS required threshold: " + std::to_string(initial_quality_threshold) + ", quality = " + std::to_string(result.quality));
				}
				
				break;
			}
			case (CalibrationResult::State::REFINING):
			{
				//refinement calibration
				//TODO:clean up
				std::vector<Transform3D> transforms;
				std::vector<float> weights;
				for (int j = 0; j < chunks.size() - 1; j++) {
					weights.push_back(100000);
					transforms.push_back(utility::calibration::Position::refineIdenticalPairPosition(chunked_pos1[j], chunked_pos2[j], result.transform, &weights.back()));
					weights.back() = utility::qualityFromError(weights.back(), qualityScaleFactor);
				}
				result.transform = getMeanTransform(transforms, weights);

				transforms.clear();
				weights.clear();
				for (int j = 0; j < chunks.size() - 1; j++) {
					weights.push_back(100000);
					transforms.push_back(utility::calibration::Position::refineIdenticalPairRotation(chunked_pos1[j], chunked_pos2[j], result.transform, &weights.back()));
					weights.back() = utility::qualityFromError(weights.back(), qualityScaleFactor);
				}
				transforms.push_back(result.transform);
				weights.push_back(result.quality);
				result.transform = getMeanTransform(transforms, weights);
				//TODO:clean up

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
				//TODO: fix fault detection for new model
				//Track new transform and see how much it moves? (expensive)
				float error = utility::calibration::Position::getError(pos1, pos2, currentCalibration.transform);
				result.relevance = result.relevance * (1 - fault_hysteresis_rate) + fault_hysteresis_rate * utility::qualityFromError(error, qualityScaleFactor);
				FUSION_LOG(" Already calibrated - watching for faults - relevance = " + std::to_string(result.relevance) + " vs. quality = " + std::to_string(result.quality));
				FUSION_LOG(" relevance / quality = " + std::to_string(result.relevance / result.quality) + " vs. thres = " + std::to_string(fault_threshold));
				//Relevance is the latest quality value, filtered with exponential filter
				//If our quality varies from the expected too much, we need to recalibrate
				if (result.relevance / result.quality < fault_threshold) {
					if (result.quality < initial_quality_threshold) {
						//Try to improve the relevance
						result.state = CalibrationResult::State::UNCALIBRATED;
					} else {
						//Try to improve the relevance
						//TODO: fix refinement
						result.state = CalibrationResult::State::UNCALIBRATED;
						result.quality = result.relevance;
					}
				}
				else {
					result.state = CalibrationResult::State::CALIBRATED;
				}
				//DEBUG: STAY CALIBRATED
				result.state = CalibrationResult::State::CALIBRATED;
				break;
			}
		}
		ss << "Result: transform[" << result.systems.first.name << "->" << result.systems.second.name << "] = " << std::endl << result.transform.matrix() << std::endl;
		FUSION_LOG(ss.str());
		return result;
	}

	CalibrationResult Calibrator::cal6DoF(const std::vector<Measurement::Ptr>& m1, const std::vector<Measurement::Ptr>& m2, const CalibrationResult& currentCalibration)const
	{
		if (currentCalibration.state == CalibrationResult::State::CALIBRATED) return currentCalibration;
		float qualityScaleFactor = 1;

		//Debug
		std::stringstream ss;
		ss << "cal6Dof[" << m1.front()->getSensor()->system.name << ", " << m2.front()->getSensor()->system.name << "]" << std::endl;


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

			ss << "stream1, group " << index << " \n"<< pos1[index].back() << std::endl;
			ss << "stream2, group " << index << " \n"<< pos2[index].back() << std::endl;
		}
		CalibrationResult result;
		result.systems = SystemPair(m1.front()->getSystem(), m2.front()->getSystem());
		
		//Compute transforms and errors
		std::vector<float> weights;
		std::vector<Transform3D> transformsX;
		std::vector<Transform3D> transformsY;
		for (int i = 0; i < pos1.size(); i++) {
			float error = 100;
			// pos1[i][k] * X = Y * pos2[i][k] 
			//Y:System2->System1
			auto group_result = utility::calibration::Transform::twoSystems_Kronecker_Shah2013(pos1[i], pos2[i], &error);
			transformsX.push_back(group_result.first);
			transformsY.push_back(group_result.second);
			weights.push_back(utility::qualityFromError(error, qualityScaleFactor));
		}
		//Compute mean transforms over each group
		Transform3D transformX = getMeanTransform(transformsX, weights);
		Transform3D transformY = getMeanTransform(transformsY, weights);
		result.transform = transformY.inverse(); //Y':System1->System2

		//Compute error
		result.error = 0;
		for (int i = 0; i < pos1.size(); i++) {
			result.error += utility::calibration::Transform::getTwoSystemsError(transformX, transformY, pos1[i], pos2[i]);
		}
		result.error = result.error / pos1.size();
		
		//TODO: compute quality
		result.quality = utility::qualityFromError(result.error, 1);
		result.state = CalibrationResult::State::CALIBRATED;

		ss << "Result: transformX = " << std::endl << transformX.matrix() << std::endl;
		ss << "Result: transformY = " << std::endl << transformY.matrix() << std::endl;
		ss << "Result: transform[" << result.systems.first.name << "->" << result.systems.second.name << "] = " << std::endl << result.transform.matrix() << std::endl;
		ss << "Result: error[" << result.systems.first.name << "->" << result.systems.second.name << "] = " << std::endl << result.error << std::endl;
		ss << "Result: quality[" << result.systems.first.name << "->" << result.systems.second.name << "] = " << std::endl << result.quality << std::endl;
		FUSION_LOG(ss.str());
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