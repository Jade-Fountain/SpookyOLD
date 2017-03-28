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
#include "FusionTypes.h"

//#define NDEBUG
#include <cassert>

namespace fusion {
	//Define config constants
	const float Measurement::uncertainty_growth_max = 0.01f; //Fractional growth per second

	Measurement::Ptr Measurement::createCartesianMeasurement(Eigen::Vector3f position, Eigen::Matrix<float,3,3> sigma) {
		Measurement::Ptr meas = std::make_shared<Measurement>();
		meas->type = MeasurementType::POSITION;
		meas->data = position;
		meas->uncertainty = sigma;
		meas->size = position.rows();
		return std::move(meas);
	}
	Measurement::Ptr Measurement::createQuaternionMeasurement(Eigen::Vector4f quaternion, Eigen::Matrix<float,4,4> sigma) {
		Measurement::Ptr meas = std::make_shared<Measurement>();
		meas->type = MeasurementType::ROTATION;
		meas->data = quaternion;
		meas->uncertainty = sigma;
		meas->size = quaternion.rows();
		return std::move(meas);
	}
	Measurement::Ptr Measurement::createScaleMeasurement(Eigen::Vector3f scale, Eigen::Matrix<float,3,3> sigma) {
		Measurement::Ptr meas = std::make_shared<Measurement>();
		meas->type = MeasurementType::SCALE;
		meas->data = scale;
		meas->uncertainty = sigma;
		meas->size = scale.rows();
		return std::move(meas);
	}
	Measurement::Ptr Measurement::createRigidBodyMeasurement(Eigen::Matrix<float,7,1> pos_quat, Eigen::Matrix<float,7,7> sigma) {
		Measurement::Ptr meas = std::make_shared<Measurement>();
		meas->type = MeasurementType::RIGID_BODY;
		meas->data = pos_quat;
		meas->uncertainty = sigma;
		meas->size = pos_quat.rows();
		return std::move(meas);
	}

	float Measurement::compare(const Measurement::Ptr& other) {
		if (type != other->type) {
			throw std::runtime_error(__FILE__ + __LINE__ + std::string(" : Cannot compare two measurements of differing type"));
		}
		//TODO: add conditionals for better metrics for rotations etc.
		return (data - other->data).norm();
	}

	//TODO: refactor using custom struct with two measurement streams
	std::vector<Measurement::Ptr> Measurement::synchronise(
		const std::vector<Measurement::Ptr>& source, 
		const std::vector<Measurement::Ptr>& target,
		std::vector<Measurement::Ptr>& target_out
	){
		std::vector<Measurement::Ptr> result;

		std::vector<Measurement::Ptr>::const_iterator source_it = source.begin();
		std::vector<Measurement::Ptr>::const_iterator target_it = target.begin();
		
		if ((*source_it)->timestamp == (*target_it)->timestamp) {
			result.push_back(*source_it);
			target_out.push_back(*target_it);
		}

		while(target_it != target.end()){
			//Iterate to target after current source
			while(
				target_it != target.end() && 
				(*target_it)->timestamp <= (*source_it)->timestamp
			){
				target_it++;
			}
			
			//If we ran out of target measurements
			if(target_it == target.end()) break;

			//Increase source iterator until the next measurement is after the current target
			while(
				std::next(source_it) != source.end() && 
				(*std::next(source_it))->timestamp < (*target_it)->timestamp
			){
				source_it++;
			}

			//If there are no more source measurements
			if(std::next(source_it) == source.end()) break;
			
			//Interpolate between nearest measurements
			std::vector<Measurement::Ptr>::const_iterator lower_source_it = source_it;
			std::vector<Measurement::Ptr>::const_iterator upper_source_it = std::next(source_it);

			float t0 = (*lower_source_it)->timestamp;
			float t1 = (*upper_source_it)->timestamp;
			float t = ((*target_it)->timestamp - t0) / (t1-t0);
			target_out.push_back(*target_it);
			result.push_back(Measurement::interpolate(*lower_source_it,*upper_source_it,t));
			
			//Place source_it after/equal to current target_it
			source_it++;
		}

		return result;
	}

	Measurement::Ptr Measurement::interpolate(const Measurement::Ptr& m0, const Measurement::Ptr& m1, float t){
		assert(m0->getSensor() == m1->getSensor());
		Measurement::Ptr result = std::make_shared<Measurement>(*m0);
		//TODO: support nonlinear data types
		result->data =  m0->data * (1-t) + m1->data * t;
		result->timestamp =  m0->timestamp * (1-t) + m1->timestamp * t;

		float uncertainty_growth = 4 * t * (1-t) * uncertainty_growth_max * (m0->timestamp - m1->timestamp) / 2;
		result->uncertainty = (m0->uncertainty * (1-t) + m1->uncertainty * t) * (1 + uncertainty_growth);
		result->confidence = m0->confidence * (1-t) + m1->confidence * t;
		return result;
	}

	Measurement::Ptr Measurement::extrapolate(const Measurement::Ptr& m, float time_sec){
		Measurement::Ptr result = std::make_shared<Measurement>(*m);
		//Grow uncertainty linearly based on the time elapsed
		float uncertainty_growth = time_sec * uncertainty_growth_max;
		result->uncertainty = (m->uncertainty) * (1 + uncertainty_growth);
		//Otherwise guess same data, etc.
		return result;
	}


}


