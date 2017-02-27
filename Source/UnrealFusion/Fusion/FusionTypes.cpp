// Fill out your copyright notice in the Description page of Project Settings.

#include "UnrealFusion.h"
#include "FusionTypes.h"

namespace fusion {

	Measurement::Ptr Measurement::createCartesianMeasurement(Eigen::Vector3f position, Eigen::Matrix<float,3,3> sigma) {
		Measurement::Ptr meas = std::make_shared<Measurement>();
		meas->type = MeasurementType::POSITION;
		meas->data = position;
		meas->uncertainty = sigma;
		meas->size = position.size;
		return std::move(meas);
	}
	Measurement::Ptr Measurement::createQuaternionMeasurement(Eigen::Vector4f quaternion, Eigen::Matrix<float,4,4> sigma) {
		Measurement::Ptr meas = std::make_shared<Measurement>();
		meas->type = MeasurementType::ROTATION;
		meas->data = quaternion;
		meas->uncertainty = sigma;
		meas->size = quaternion.size;
		return std::move(meas);
	}
	Measurement::Ptr Measurement::createScaleMeasurement(Eigen::Vector3f scale, Eigen::Matrix<float,3,3> sigma) {
		Measurement::Ptr meas = std::make_shared<Measurement>();
		meas->type = MeasurementType::SCALE;
		meas->data = scale;
		meas->uncertainty = sigma;
		meas->size = scale.size;
		return std::move(meas);
	}
	Measurement::Ptr Measurement::createRigidBodyMeasurement(Eigen::Matrix<float,7,1> pos_quat, Eigen::Matrix<float,7,7> sigma) {
		Measurement::Ptr meas = std::make_shared<Measurement>();
		meas->type = MeasurementType::RIGID_BODY;
		meas->data = pos_quat;
		meas->uncertainty = sigma;
		meas->size = pos_quat.size;
		return std::move(meas);
	}

}


