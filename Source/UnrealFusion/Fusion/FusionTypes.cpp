// Fill out your copyright notice in the Description page of Project Settings.

#include "UnrealFusion.h"
#include "FusionTypes.h"

Measurement Measurement::createPositionMeasurement(Eigen::Vector3f position, Eigen::Matrix<float,3,3> sigma) {
	Measurement meas;
	meas.type = MeasurementType::POSITION;
	meas.data = position;
	meas.uncertainty = sigma;
	return meas;
}
Measurement Measurement::createRotationMeasurement(Eigen::Vector4f quaternion, Eigen::Matrix<float,4,4> sigma) {
	Measurement meas;
	meas.type = MeasurementType::ROTATION;
	meas.data = quaternion;
	meas.uncertainty = sigma;
	return meas;
}
Measurement Measurement::createScaleMeasurement(Eigen::Vector3f scale, Eigen::Matrix<float,3,3> sigma) {
	Measurement meas;
	meas.type = MeasurementType::SCALE;
	meas.data = scale;
	meas.uncertainty = sigma;
	return meas;
}
Measurement Measurement::createRigidBodyMeasurement(Eigen::Vector<float,7,1> pos_quat, Eigen::Matrix<float,7,7> sigma) {
	Measurement meas;
	meas.type = MeasurementType::RIGID_BODY;
	meas.data = pos_quat;
	meas.uncertainty = sigma;
	return meas;
}

SensorNode::SensorNode()
{
}

SensorNode::~SensorNode()
{
}

