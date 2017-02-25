// Fill out your copyright notice in the Description page of Project Settings.

#pragma once
// Fill out your copyright notice in the Description page of Project Settings.

#include <string>
#include <queue>
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
	//Accessors
	const Eigen::MatrixXf& getUncertainty() const { return uncertainty; }
	const Eigen::VectorXf& getData() const { return data; }

	//Name of the sensor system from which the measurement came
	std::string systemName = "";

	//Sensor number corresponding to measurement
	int sensorID = 0;

	//Timestamp (sec; from device)
	double timeStamp = -1;

	//Confidence in T in [0,1]
	float confidence = 0;
	
	//Setup Methods
	bool check_consistent() {
		return (size == data.size() == uncertainty.rows() == uncertainty.cols());
	}

	bool setMetaData(std::string system_name, int sensor_id, float timestamp_sec, float confidence_){
		systemName = system_name;
		sensorID = sensor_id;
		timeStamp = timestamp_sec;
		confidence = confidence_;
		return check_consistent();
	}


	//Static factory methods:
	static Measurement createCartesianMeasurement(Eigen::Vector3f position, Eigen::Matrix<float,3,3> sigma);
	static Measurement createQuaternionMeasurement(Eigen::Vector4f quaternion, Eigen::Matrix<float,4,4> sigma);
	static Measurement createScaleMeasurement(Eigen::Vector3f scale, Eigen::Matrix<float,3,3> sigma);
	static Measurement createRigidBodyMeasurement(Eigen::Matrix<float,7,1> pos_quat, Eigen::Matrix<float,7,7> sigma);
};

//NOTE: templating means each skeleton is of a particular model type1
class CartesianModel{
	//Data model for the state of the data
	struct State{
	public:
		Eigen::Vector3f expectation;
		Eigen::Matrix<float,3,3> uncertainty;
	};
	
	static void updateState(State* state, Measurement measurement){
		//Use latest measurement
		state->expectation = measurement.getData().segment(0,3);//start,count
		state->uncertainty = measurement.getUncertainty().topLeftCorner(3,3);
	}
};


/**
 * Sensor Nodes model the state of the system with a tree of nodes
 */
template <typename Model>
class SensorNode
{
private:
	//Current best state estimate, typically including some estimate of variance or confidence
	typename Model::State state;
	
	//Queued messages, 
	//TODO: order by timestamp
	std::queue<Measurement> measurements;

	//Children of this node
	std::vector<int> children_indices;

	//Parent of this node
	int parent_index;

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

	SensorNode() {}
	~SensorNode() {}

};


typedef SensorNode<CartesianModel> DefaultSensorNode;
