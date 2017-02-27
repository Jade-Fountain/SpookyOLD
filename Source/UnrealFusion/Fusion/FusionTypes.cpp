// Fill out your copyright notice in the Description page of Project Settings.

#include "UnrealFusion.h"
#include "FusionTypes.h"

namespace fusion {

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

}


