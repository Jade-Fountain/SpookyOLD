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

#include <string>
#include <queue>
#include <set>
#include <memory>
#include "Eigen/Core"
#include "Eigen/Geometry"
#include "Logging.h"
#include "Utilities/DataStructures.h"
#pragma once


namespace fusion {

	//For convenience and abstraction, typedef some basic structs

	//Mapping between two affine spaces
	typedef Eigen::Transform<float, 3, Eigen::Affine> Transform3D;



	/** System descriptor - abstraction for a string type to be used as a map key - might be changed later
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

	//Node descriptor uses the same class as systemdescriptor
	typedef SystemDescriptor NodeDescriptor;

	//Map key types
	typedef std::pair<SystemDescriptor, SystemDescriptor> SystemPair;
	typedef std::pair<SystemDescriptor, NodeDescriptor> SystemNodePair;
	
	//Sensor ID type
	typedef int SensorID;

	//Function to overload system pair map keying
	struct SystemPairCompare {
		bool operator() (const SystemPair& lhs, const SystemPair& rhs) {
			std::string lhs_combined = lhs.first.name + lhs.second.name;
			std::string rhs_combined = rhs.first.name + rhs.second.name;
			return lhs_combined < rhs_combined;
		}
	};


	//Results of a calibration are stored in this struct and returned
	class CalibrationResult {
	public:
		enum State {
			UNCALIBRATED = 0,
			REFINING = 1,
			CALIBRATED = 2
		};

		//systems = [domain,range]
		SystemPair systems;
		//Calibration state describes
		State state = UNCALIBRATED;
		//Maps systems.first to systems.second
		Transform3D transform;
		//Error is the mean re-projection error
		float error = 0;
		//Quality is a qualitative measure in [0,1] of the estimated accuracy of the result
		float quality = 0;

		//Relevance - parameter used to detect when a system config has changed
		float relevance = 1;

		//Returns the inverse of the calibration result
		CalibrationResult inverse() {
			CalibrationResult result = *this;
			result.transform = transform.inverse();
			return result;
		}

		//Checks if sensor is calibrated
		bool calibrated() {
			return state != UNCALIBRATED;
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

	//Sensor describes all persistent parameters of a given sensor
	//WARNING TO DEVS: Measurement::Ptr cannot be stored in here or a memory leak will occur (shared ptr loop)
	class Sensor {
	public:
		//=================================================
		//Name of the sensor system from which the measurement came
		SystemDescriptor system;

		//Sensor number corresponding to measurement
		SensorID id = 0;

		//Possible nodes which this sensor is attached to
		std::set<NodeDescriptor> nodes;
		//TODO: Set of eliminated nodes
		std::set<NodeDescriptor> eliminatedNodes;
		//=================================================

		//Typedef ptr to this class for neater code later
		typedef std::shared_ptr<Sensor> Ptr;

		//Accessors:
		bool isResolved() { return utility::setDiff(nodes, eliminatedNodes).size() == 1; }
		bool isAmbiguous() { return nodes.size() != 1; }

		//Returns a valid node only when there is only one possibility
		NodeDescriptor getNode() {
			std::set<NodeDescriptor> nodesFinal = utility::setDiff(nodes, eliminatedNodes);
			if (nodesFinal.size() != 1) {
				FUSION_LOG(__FILE__ + __LINE__ + std::string(" : attempted to get node of ambiguous sensor"));
				return "__AMBIGUOUS__"; 
			}
			return *nodesFinal.begin();
		}

		//gets all possible nodes
		std::set<NodeDescriptor> getNodes() {
			return nodes;
		}

		std::set<NodeDescriptor> getRemainingNodes(){
			return utility::setDiff(nodes,eliminatedNodes);
		}
		
		//Adds a node as a possible sensor location
		void addNode(const NodeDescriptor& node) {
			nodes.insert(node);
		}

		bool nodeEliminated(const NodeDescriptor& node){
			return eliminatedNodes.count(node) > 0 || nodes.count(node) < 1;
		}

		void eliminateNode(const NodeDescriptor& node){
			eliminatedNodes.insert(node);
		}

	};

	//Class describing individual sensor reading taken at a particular time
	class Measurement {
	public:
		typedef std::shared_ptr<Measurement> Ptr;

		//=========================
		//			Members
		//=========================
	private:		
		//Measurement dimensions
		int size;
		//Value of measurement
		Eigen::VectorXf data;

		//Uncertainty in T
		Eigen::MatrixXf uncertainty;

		//Sensor information
		Sensor::Ptr sensor;
	public:

		//Timestamp (sec; from device)
		//TODO: ensure double precision input
		double timestamp = -1;

		//Confidence in T in [0,1]
		float confidence = 0;

		//Type of measurement
		MeasurementType type;

		//=========================
		//			Methods
		//=========================

		//Setup Methods
		bool check_consistent() {
			return (size == data.size() == uncertainty.rows() == uncertainty.cols());
		}

		bool setMetaData(float timestamp_sec, float confidence_){
			timestamp = timestamp_sec;
			confidence = confidence_;
			return check_consistent();
		}

		//=========================
		//Static factory methods:
		//=========================
		static Measurement::Ptr createCartesianMeasurement(Eigen::Vector3f position, Eigen::Matrix<float,3,3> sigma);
		static Measurement::Ptr createQuaternionMeasurement(Eigen::Vector4f quaternion, Eigen::Matrix<float,4,4> sigma);
		static Measurement::Ptr createScaleMeasurement(Eigen::Vector3f scale, Eigen::Matrix<float,3,3> sigma);
		static Measurement::Ptr createRigidBodyMeasurement(Eigen::Matrix<float,7,1> pos_quat, Eigen::Matrix<float,7,7> sigma);


		//=========================
		//Data helpers
		//=========================
		
		//Method for getting the distance between two measurements
		float compare(const Measurement::Ptr & other);

		//Synchronises the source stream with the target stream
		// It is assumed that the two  streams are chronologically sorted
		static std::vector<Measurement::Ptr> synchronise(const std::vector<Measurement::Ptr>& source, 
														 const std::vector<Measurement::Ptr>& target,
														 std::vector<Measurement::Ptr>& source_out);

		//Interpolates between two measurements of the same type
		static const float uncertainty_growth_max;
		static Measurement::Ptr interpolate(const Measurement::Ptr& m0, const Measurement::Ptr& m1, float t);
		static Measurement::Ptr extrapolate(const Measurement::Ptr& m, float time_sec);

		//----------------------
		//Accessors
		//----------------------
		NodeDescriptor getNode() {
			return sensor->getNode();
		}
		std::set<NodeDescriptor> getNodes() {
			return sensor->getNodes();
		}
		SystemDescriptor getSystem() {
			return sensor->system;
		}
		SensorID getSensorID() {
			return sensor->id;
		}

		void setSensor(Sensor::Ptr& sensor_) {
			sensor = sensor_;
		}
		Sensor::Ptr getSensor() {
			return sensor;
		}

		//Accessors
		const Eigen::MatrixXf& getUncertainty() const { return uncertainty; }
		const Eigen::VectorXf& getData() const { return data; }

		bool isAmbiguous(){
			return sensor->isAmbiguous();
		}
	};


}
