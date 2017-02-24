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
	Eigen::VectorXf data;

	//Uncertainty in T
	Eigen::MatrixXf uncertainty;

public:
	//Name of the sensor system from which the measurement came
	std::string systemName = "";

	//Sensor number corresponding to measurement
	uint16 sensorID = 0;

	//Timestamp (sec; from device)
	double timeStamp = -1;

	//Confidence in T in [0,1]
	float confidence = 0;

	bool check_consistent() {
		return (size == data.size() == uncertainty.size()[0]);
	}

	static Measurement createCartesianMeasurement(Eigen::Vector3f position, Eigen::Matrix<float,3,3> sigma);
	static Measurement createQuaternionMeasurement(Eigen::Vector4f quaternion, Eigen::Matrix<float,4,4> sigma);
	static Measurement createScaleMeasurement(Eigen::Vector3f scale, Eigen::Matrix<float,3,3> sigma);
	static Measurement createRigidBodyMeasurement(Eigen::Matrix<float,7,1> pos_quat, Eigen::Matrix<float,7,7> sigma);
};

//NOTE: templating means each skeleton is of a particular model type1
class PositionModel{
	typedef Eigen::Vector3f State;
	typedef Eigen::Matrix<float,3,3> Uncertainty;

	static void updateState(State* state, Uncertainty* uncertainty, measurement){
		//Use latest measurement
		state = measurement.data.segment(0,3);//start,count
		uncertainty = measurement.uncertainty.topLeftCorner(3,3);
	}
};


/**
 * Sensor Nodes model the state of the system with a tree of nodes
 */
template <class Model>
class SensorNode
{
private:
	//Current best state estimate and variance
	Model::State state;
	Model::Uncertainty variance;
	
	//Queued messages, 
	//TODO: in order of timestamp
	std::queue<Measurement> measurements;

	//Children of this node
	std::vector<uint> children_indices;

	//Parent of this node
	uint parent_index;

public:
	//Called by parent node
	void update(){
		updateState();
		for(auto& child : children){
			child.update();
		}
	}

	//Computes update according to measurements
	void updateState(){
		while(!measurements.empty()) {
			auto& measurement = measurements.pop_front();
			Model::updateState(&state, &uncertainty, measurement);
		}
	}

	SensorNode();
	~SensorNode();

};


typedef SensorNode<PositionModel> DefaultSensorNode;
