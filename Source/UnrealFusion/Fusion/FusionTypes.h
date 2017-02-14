// Fill out your copyright notice in the Description page of Project Settings.

#pragma once
// Fill out your copyright notice in the Description page of Project Settings.

#include <string>
#include "Eigen/Core"
#pragma once

/** Structs describing measurements
*
*/
enum MeasurementType{
	GENERIC = 0,
	POSITION = 1,
	ROTATION = 2,
	RIGID_BODY = 3	
}

//TODO: make this class a parent of different measurement types
struct Measurement {

	MeasurementType type;

	//Name of the sensor system from which the measurement came
	std::string systemName = "";

	//Sensor number corresponding to measurement
	uint16 sensorID = 0;

	//Timestamp (sec; from device)
	double timeStamp;

	//Measurement dimensions
	static const uint16 size = T::size;

	//Value of measurement
	Eigen::Matrix<float, size, 1> data;

	//Uncertainty in T
	Eigen::Matrix<float, size, size> uncertainty;

	//Confidence in T in [0,1]
	float confidence;

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
};
