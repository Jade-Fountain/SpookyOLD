// Fill out your copyright notice in the Description page of Project Settings.

#pragma once
// Fill out your copyright notice in the Description page of Project Settings.

#include <string>
#include "Eigen/Core"
#pragma once

/** Structs describing measurements
*
*/
enum class MeasurementType {
	GENERIC = 0,
	POSITION = 1,
	ROTATION = 2,
	RIGID_BODY = 3,
	SCALE = 4
};

//TODO: make this class a parent of different measurement types
class Measurement {

	//Type of measurement
	MeasurementType type;

	//Measurement dimensions
	uint16 size;

	//Value of measurement
	Eigen::Matrix<float, Eigen::Dynamic, 1> data;

	//Uncertainty in T
	Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> uncertainty;

public:
	//Name of the sensor system from which the measurement came
	std::string systemName = "";

	//Sensor number corresponding to measurement
	uint16 sensorID = 0;

	//Timestamp (sec; from device)
	double timeStamp;

	//Confidence in T in [0,1]
	float confidence;

	bool check_consistent() {
		return (size == data.size() == uncertainty.size());
	}

	static Measurement createCartesianMeasurement(Eigen::Vector3f position, Eigen::Matrix<float,3,3> sigma);
	static Measurement createQuaternionMeasurement(Eigen::Vector4f quaternion, Eigen::Matrix<float,4,4> sigma);
	static Measurement createScaleMeasurement(Eigen::Vector3f scale, Eigen::Matrix<float,3,3> sigma);
	static Measurement createRigidBodyMeasurement(Eigen::Matrix<float,7,1> pos_quat, Eigen::Matrix<float,7,7> sigma);
};



/**
 * Classes for sensors
 */
class UNREALFUSION_API SensorNode
{
private:
	FTransform pose;
	FTransform uncertainty;
public:
	SensorNode();
	~SensorNode();

	//void FuseWithInversion(){
	//	std::vector<Measurement*> measurements;
	//	for(auto& measurement : measurements){//In temporal order
	//		Transform measured_pose = measurement->inverseMeasurement();
	//	}
	//}
	//void Fuse(){
	//	std::vector<Measurement*> measurements;
	//	for(auto& measurement : measurements){//In temporal order
	//		measurement->measure(pose);
	//	}
	//}
};
