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
#include "Utilities/Conventions.h"

//#define NDEBUG
#include <cassert>

namespace fusion {
	//Define config constants
	const float Measurement::uncertainty_growth_max = 0.01f; //Fractional growth per second

	//=========================
	//Static factory methods:
	//=========================
	Measurement::Ptr Measurement::createCartesianMeasurement(Eigen::Vector3f position, Eigen::Matrix<float,3,3> sigma) {
		Measurement::Ptr meas = std::make_shared<Measurement>();
		meas->type = Type::POSITION;
		meas->data = position;
		meas->uncertainty = sigma;
		meas->size = position.rows();
		return std::move(meas);
	}
	Measurement::Ptr Measurement::createQuaternionMeasurement(Eigen::Quaternionf quaternion, Eigen::Matrix<float,4,4> sigma) {
		Measurement::Ptr meas = std::make_shared<Measurement>();
		meas->type = Type::ROTATION;
		meas->data = quaternion.coeffs();
		meas->uncertainty = sigma;
		meas->size = 4;
		return std::move(meas);
	}
	Measurement::Ptr Measurement::createScaleMeasurement(Eigen::Vector3f scale, Eigen::Matrix<float,3,3> sigma) {
		Measurement::Ptr meas = std::make_shared<Measurement>();
		meas->type = Type::SCALE;
		meas->data = scale;
		meas->uncertainty = sigma;
		meas->size = scale.rows();
		return std::move(meas);
	}
	Measurement::Ptr Measurement::createPoseMeasurement(Eigen::Vector3f position, Eigen::Quaternionf quaternion, Eigen::Matrix<float,7,7> sigma) {
		Measurement::Ptr meas = std::make_shared<Measurement>();
		meas->type = Type::RIGID_BODY;
		meas->data = Eigen::Matrix<float, 7, 1>();
		meas->data << position, quaternion.coeffs();
		meas->uncertainty = sigma;
		meas->size = 7;
		return std::move(meas);
	}

	//=========================
	//Data Out Interface
	//=========================

	Eigen::Vector3f Measurement::getPosition(){
		if(type == Type::POSITION || type == Type::RIGID_BODY){
			return data.head(3);
		} else {
			return Eigen::Vector3f::Zero();
		}
	}

	Eigen::Matrix3f Measurement::getPositionVar(){
		if(type == Type::POSITION || type == Type::RIGID_BODY){
			return uncertainty.topLeftCorner(3,3);
		} else {
			return max_var * Eigen::Matrix3f::Identity();
		}
	}

	Eigen::Quaternionf Measurement::getRotation(){
		if(type == Type::ROTATION || type == Type::RIGID_BODY){
			return Eigen::Quaternionf(Eigen::Vector4f(data.tail(4)));
		} else {
			return Eigen::Quaternionf::Identity();
		}
	}

	Eigen::Matrix4f Measurement::getRotationVar(){
		if(type == Type::ROTATION || type == Type::RIGID_BODY){
			return uncertainty.bottomRightCorner(4,4);
		} else {
			return max_var * Eigen::Matrix4f::Identity();
		}
	}

	Eigen::Matrix<float,7,1>  Measurement::getPosQuat(){
		auto pos = getPosition();
		auto quat = getRotation();
		Eigen::Matrix<float,7,1> result;
		result << pos, quat.coeffs();
		return result;
	}

	Eigen::Matrix<float,7,7> Measurement::getPosQuatVar(){
		if(type == RIGID_BODY){
			return uncertainty;
		} else {
			Eigen::Matrix3f pVar = getPositionVar();
			Eigen::Matrix4f qVar = getRotationVar();
			Eigen::Matrix<float,7,7> result = Eigen::Matrix<float,7,7>::Identity();
			result.topLeftCorner(3,3) = pVar; 
			result.bottomRightCorner(4,4) = qVar;
			return result; 
		}
	}

	Transform3D Measurement::getTransform(){
		Transform3D T = Transform3D::Identity();
		bool rigid = type == Type::RIGID_BODY;
		bool pos = type == Type::POSITION || rigid;
		bool rot = type == Type::ROTATION || rigid;
		if(pos){
			T.translate(Eigen::Vector3f(data.head(3)));
		}
		if(rot){
			//Quat defined by tail 4 always
			Eigen::Quaternionf q(Eigen::Vector4f(data.tail(4)));
			T.rotate(q);
		}
		return T;
	}
	

	float Measurement::compare(const Measurement::Ptr& other) {
		if (type != other->type) {
			throw std::runtime_error(__FILE__ + __LINE__ + std::string(" : Cannot compare two measurements of differing type"));
		}
		//TODO: deal with noisy measurements, esp rotation
		if (type == Type::POSITION || type == Type::RIGID_BODY) {
			return (getPosition() - other->getPosition()).norm();
		}
		else if (type == Type::ROTATION) {
			return Eigen::AngleAxisf(getRotation().inverse() * other->getRotation()).angle();
		}
		else {
			return (data - other->data).norm();
		}
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

			//Avoid interpolating if possible
			//TODO: add small threshold?
			if ((*lower_source_it)->timestamp == (*target_it)->timestamp) {
				result.push_back(*lower_source_it);
				target_out.push_back(*target_it);
			} else {
				//Interpolate to synchronise
				float t0 = (*lower_source_it)->timestamp;
				float t1 = (*upper_source_it)->timestamp;
				float t = ((*target_it)->timestamp - t0) / (t1 - t0);
				target_out.push_back(*target_it);

				result.push_back(Measurement::interpolate(*lower_source_it, *upper_source_it, t));
			}

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


