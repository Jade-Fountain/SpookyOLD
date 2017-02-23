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

public:
	MeasurementType type;

	//Name of the sensor system from which the measurement came
	std::string systemName = "";

	//Sensor number corresponding to measurement
	uint16 sensorID = 0;

	//Timestamp (sec; from device)
	double timeStamp;

	//Measurement dimensions
	uint16 size;

	//Value of measurement
	Eigen::Matrix<float, Eigen::Dynamic,1> data;

	//Uncertainty in T
	Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> uncertainty;

	//Confidence in T in [0,1]
	float confidence;

	bool check_consistent() {
		return (size == data.size() == uncertainty.size());
	}

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
