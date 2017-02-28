#include "UnrealFusion.h"
#include "Calibration.h"
#include "Fusion/Utilities/CalibrationUtilities.h"

namespace fusion {
	/////////////////////////////////////////////////////////////////////////////////////////////////////////
	//									CalibrationDataSet
	/////////////////////////////////////////////////////////////////////////////////////////////////////////

	void CalibrationDataSet::Stream::addMeasurement(const Measurement::Ptr& m) {
		safeAccess(sensors,m->sensorID).push_back(m);
	}

	std::pair<SensorID, size_t> CalibrationDataSet::Stream::maxCount()
	{
		std::pair<SensorID, size_t> result(0,0);
		for (auto& sensor : sensors) {
			size_t s = sensor.second.size();
			if (result.second < s) {
				result.second = s;
				result.first = sensor.first;
			}
		}
		return result;
	}

	void CalibrationDataSet::addMeasurement(const Measurement::Ptr& m, const SystemDescriptor& system, const NodeDescriptor& node) {
		SystemNodePair sysNode = SystemNodePair(system, node);
		systemNodeTable[sysNode].addMeasurement(m);
		//Store info for later
		systems.insert(system);
		nodes.insert(node);
	}

	float CalibrationDataSet::compareMeasurement(const Measurement::Ptr & m, const SystemDescriptor & system, const NodeDescriptor & node)
	{
		SystemNodePair sysNode = SystemNodePair(system, node);
		std::vector<Measurement::Ptr>& stream = safeAccess(systemNodeTable[sysNode].sensors,m->sensorID);
		//If no previous recorded data, return max difference
		if (stream.size() == 0) {
			return float(std::numeric_limits<float>::max());
		}
		//Otherwise compare the measurements
		return stream.back()->compare(m);
	}



	/////////////////////////////////////////////////////////////////////////////////////////////////////////
	//									Calibrator:Private
	/////////////////////////////////////////////////////////////////////////////////////////////////////////

	std::vector<std::pair<Measurement::Ptr, NodeDescriptor>>
	Calibrator::filterLonelyData(const std::vector<std::pair<Measurement::Ptr, NodeDescriptor>>& measurementQueue) {
		//Init result
		std::vector<std::pair<Measurement::Ptr, NodeDescriptor>> result;
		//Structure for counting systems per node
		SafeMap<NodeDescriptor, std::set<SystemDescriptor>> systemsPerNode;
		//Count
		for (auto& m : measurementQueue) {
			systemsPerNode[m.second].insert(m.first->system);
		}
		//Push back relevant measurments
		for (auto& m : measurementQueue) {
			if (systemsPerNode[m.second].size() > 1) {
				result.push_back(m);
			}
		}
		return result;
	}

	bool Calibrator::checkChanges(const std::vector<std::pair<Measurement::Ptr, NodeDescriptor>>& measurements) {
		//Check change for each measurement
		for (auto& m : measurements) {
			auto& mes = m.first;
			auto& node = m.second;
			float diff = calibrationSet.compareMeasurement(mes, mes->system, node);
			//TODO:Perform next check over each node individually
			//If any of the measurements are new then return true
			if (diff > diff_threshold) {
				return true;
			}
		}
		return false;
	}


	CalibrationResult Calibrator::calibrateStreams(const std::vector<Measurement::Ptr>& m1, const std::vector<Measurement::Ptr>& m2)
	{
		switch (m1.front()->type) {
		case MeasurementType::POSITION:
			if (m2.front()->type == MeasurementType::POSITION) {
				return calPosPos(m1, m2);
			}
			break;
		}
		return CalibrationResult();
	}

	CalibrationResult Calibrator::calPosPos(const std::vector<Measurement::Ptr>& m1, const std::vector<Measurement::Ptr>& m2)
	{
		std::vector<Eigen::Vector3f> pos1(m1.size());
		std::vector<Eigen::Vector3f> pos2(m2.size());
		for (int i = 0; i < m1.size(); i++) {
			pos1[i] = m1[i]->getData();
			pos2[i] = m2[i]->getData();
		}
		CalibrationResult result;
		result.systems = SystemPair(m1.front()->system, m2.front()->system);
		result.transform = utility::PositionalCalibration::calibrateIdenticalPair(pos1, pos2);
		result.quality = 1;//TODO: utility::PositionalCalibration::checkError(pos1, pos2, result.transform);
		result.calibrated = true;
		return result;
	}
	/////////////////////////////////////////////////////////////////////////////////////////////////////////
	//									Calibrator:Public
	/////////////////////////////////////////////////////////////////////////////////////////////////////////
	void Calibrator::addMeasurement(const Measurement::Ptr& m, const NodeDescriptor& node) {
		calibrationSet.addMeasurement(m, m->system, node);
	}

	void Calibrator::addMeasurementGroup(const std::vector<std::pair<Measurement::Ptr, NodeDescriptor>>& measurementQueue) {
		
		//Check there is data corresponding to more than one system for a given node, otherwise useless
		auto measurements = filterLonelyData(measurementQueue);
		
		//Decide if data is useful
		bool dataNovel = checkChanges(measurements);

		if (dataNovel) {
			//Store the (refs to) the relevant measurements
			for (auto& m : measurements) {
				addMeasurement(m.first, m.second);
			}

		}
	}

	void Calibrator::calibrate()
	{
		std::map < NodeDescriptor, std::vector<SystemDescriptor>> calibrationGroup;

		//For each unordered pairing of systems, check if there are common nodes
		for (std::set<SystemDescriptor>::iterator system1 = calibrationSet.systems.begin(); system1 != calibrationSet.systems.end(); system1++) {
			for (std::set<SystemDescriptor>::iterator system2 = std::next(system1); system2 != calibrationSet.systems.end(); system2++) {
				
				//Init list of calibration results: on result for each node
				std::vector<CalibrationResult> results;
				//Create key for the pair of systems
				SystemPair sysPair(*system1,*system2);

				//Loop through nodes and calibrate with them if they provide relevant info
				for (auto& node : calibrationSet.nodes) {
					
					//Keys for accessing data streams
					SystemNodePair sysNode1(*system1, node);
					SystemNodePair sysNode2(*system2, node);
					
					//If there is an entry for each system in the table, add the systems to the node calibration group
					if (calibrationSet.systemNodeTable.count(sysNode1) > 0 &&
						calibrationSet.systemNodeTable.count(sysNode2) > 0) 
					{
						//TODO:Check data counts
						std::pair<SensorID, size_t> max1 = calibrationSet.systemNodeTable[sysNode1].maxCount();
						std::pair<SensorID, size_t> max2 = calibrationSet.systemNodeTable[sysNode2].maxCount();

						//Streams of different length - we cant use this data
						if (max1.second != max2.second && max1.second < count_threshold && max2.second < count_threshold) {
							//TODO: do something
							continue; //cannot calibrate this pair of sensors
						}

						//Get measurements
						const std::vector<Measurement::Ptr>& measurements1 = calibrationSet.systemNodeTable[sysNode1].sensors[max1.first];
						const std::vector<Measurement::Ptr>& measurements2 = calibrationSet.systemNodeTable[sysNode2].sensors[max2.first];

						//Perform calibration
						results.push_back(calibrateStreams(measurements1, measurements2));

					}
				}

				//TODO: result = combineResults(results); // combine multiple results
				//Store results
				if (results.size() > 0) {
					calibrationResults[sysPair] = results.front();
				}

			}
		}
	}

	CalibrationResult Calibrator::getResultsFor(SystemDescriptor s1, SystemDescriptor s2)
	{
		SystemPair forward(s1, s2);
		SystemPair reverse(s1, s2);
		if (calibrationResults.count(forward) > 0) {
			return calibrationResults[forward];
		}
		if(calibrationResults.count(reverse) > 0) {
			return calibrationResults[reverse].inverse();
		}
		return CalibrationResult();
	}

}
