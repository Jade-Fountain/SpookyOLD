// Fill out your copyright notice in the Description page of Project Settings.

#pragma once
// Fill out your copyright notice in the Description page of Project Settings.

#include <string>
#include <queue>
#include <memory>
#include "Eigen/Core"
#include "Eigen/Geometry"
#pragma once


namespace fusion {

	//Basic types:
	typedef Eigen::Transform<float, 3, Eigen::Affine> Transform3D;


	/** System descriptor
	*
	*/
	class SystemDescriptor {
	public:
		//Name of the system - used for checking system identity
		std::string name;

		//Overloaded operators check for valid system name and check equality
		bool SystemDescriptor::operator==(const SystemDescriptor &other) const {
			return name.size() > 0 && other.name.size() > 0 && name.compare(other.name) == 0;  // Compare the values, and return a bool result.
		}
		bool SystemDescriptor::operator!=(const SystemDescriptor &other) const {
			return !(*this == other);
		}
		//Comparators for maps
		bool operator<(const SystemDescriptor& rhs) const { return name < rhs.name; }
		bool operator>(const SystemDescriptor& rhs) const { return name > rhs.name; }
		//Constructor
		SystemDescriptor(std::string n = "") : name(n) {}
	};

	/*class NodeDescriptor : public SystemDescriptor {
	public:
		NodeDescriptor(std::string n = "") : SystemDescriptor(n) {}
	};*/
	typedef SystemDescriptor NodeDescriptor;

	typedef std::pair<SystemDescriptor, SystemDescriptor> SystemPair;
	typedef std::pair<SystemDescriptor, NodeDescriptor> SystemNodePair;
	typedef int SensorID;

	struct SystemPairCompare {
		bool operator() (const SystemPair& lhs, const SystemPair& rhs) {
			std::string lhs_combined = lhs.first.name + lhs.second.name;
			std::string rhs_combined = rhs.first.name + rhs.second.name;
			return lhs_combined < rhs_combined;
		}
	};


	//Results of a calibration are stored in this struct
	class CalibrationResult {
	public:
		SystemPair systems;
		bool calibrated = false;
		//Maps first to second
		Transform3D transform;
		float quality = 0;

		CalibrationResult inverse() {
			CalibrationResult result = *this;
			result.transform = transform.inverse();
			return result;
		}
	};

	/** Structs describing measurements
	*
	*/
	enum MeasurementType {
		GENERIC = 0,
		POSITION = 1,
		ROTATION = 2,
		RIGID_BODY = 3,
		SCALE = 4
	};

	//TODO: make this class a parent of different measurement types
	class Measurement {
	public:
		//Type of measurement
		MeasurementType type;

		//Measurement dimensions
		int size;
	private:
		//Value of measurement
		Eigen::VectorXf data;

		//Uncertainty in T
		Eigen::MatrixXf uncertainty;

	public:
		//Accessors
		const Eigen::MatrixXf& getUncertainty() const { return uncertainty; }
		const Eigen::VectorXf& getData() const { return data; }

		//Name of the sensor system from which the measurement came
		SystemDescriptor system;

		//Sensor number corresponding to measurement
		SensorID sensorID = 0;

		//Timestamp (sec; from device)
		double timeStamp = -1;

		//Confidence in T in [0,1]
		float confidence = 0;
		
		//Setup Methods
		bool check_consistent() {
			return (size == data.size() == uncertainty.rows() == uncertainty.cols());
		}

		bool setMetaData(SystemDescriptor system_name, int sensor_id, float timestamp_sec, float confidence_){
			system = system_name;
			sensorID = sensor_id;
			timeStamp = timestamp_sec;
			confidence = confidence_;
			return check_consistent();
		}
		
		typedef std::shared_ptr<Measurement> Ptr;

		//Static factory methods:
		static Measurement::Ptr createCartesianMeasurement(Eigen::Vector3f position, Eigen::Matrix<float,3,3> sigma);
		static Measurement::Ptr createQuaternionMeasurement(Eigen::Vector4f quaternion, Eigen::Matrix<float,4,4> sigma);
		static Measurement::Ptr createScaleMeasurement(Eigen::Vector3f scale, Eigen::Matrix<float,3,3> sigma);
		static Measurement::Ptr createRigidBodyMeasurement(Eigen::Matrix<float,7,1> pos_quat, Eigen::Matrix<float,7,7> sigma);
		
		//Method for getting the distance between two measurements
		float compare(const Measurement::Ptr & other);
	};


}
