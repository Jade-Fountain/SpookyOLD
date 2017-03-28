
#include "UnrealFusion.h"
#include "Correlator.h"

namespace fusion {
	//---------------------------------------------------------------------------------
	//										Calibrator::Data
	//---------------------------------------------------------------------------------

	void Correlator::Data::addAmbiguous(const Measurement::Ptr& m) {
		Sensor::Ptr sensor = m->getSensor();
		//If sensor stream not initialised
		if (ambiguous_measurements.sensors.count(sensor) == 0) {
			//Initialise ambiguous measurements for sensor
			ambiguous_measurements.sensors[sensor] = std::vector<Measurement::Ptr>();
			const std::set<NodeDescriptor>& nodes = sensor->getNodes();
			//Store relevant nodes for later in the add unambiguous measurement function
			relevant_nodes.insert(nodes.begin(), nodes.end());
		} else {
			//Simply add measurement
			ambiguous_measurements.sensors[sensor].push_back(m);
		}
	}

	void Correlator::Data::addUnambiguous(const Measurement::Ptr& m) {
		Sensor::Ptr sensor = m->getSensor();
		utility::safeAccess(
			utility::safeAccess(unambiguous_measurements, m->getNode()).sensors,
			sensor
			).push_back(m);
	}

	bool Correlator::Data::unambiguousMeasurementNeeded(const Sensor::Ptr& s) {
		return relevant_nodes.count(s->getNode()) > 0;
	}

	float Correlator::Data::compareMeasurement(const Measurement::Ptr& m){
		Sensor::Ptr sensor = m->getSensor();
		if(unseen(sensor)) return float(std::numeric_limits<float>::max());
		if(m->isAmbiguous()){
			return ambiguous_measurements.sensors[sensor].back()->compare(m);
		} else {
			return unambiguous_measurements[m->getNode()].sensors[sensor].back()->compare(m);
		}
	}

	bool Correlator::Data::unseen(const Sensor::Ptr& sensor){
		if(sensor->isAmbiguous()){
			return utility::safeAccess(ambiguous_measurements.sensors,sensor).size() == 0;
		} else {
			return utility::safeAccess(
				utility::safeAccess(unambiguous_measurements, sensor->getNode()).sensors,
				sensor
			).size() == 0;
		}
	}

	
	const Correlator::Data::Streams& Correlator::Data::getUnambiguousStreams(const NodeDescriptor & node)
	{
		return utility::safeAccess(unambiguous_measurements,node);
	}
	
	int Correlator::Data::ambiguousCount(const Sensor::Ptr& sensor){
		return utility::safeAccess(ambiguous_measurements.sensors,sensor).size();
	}

	void Correlator::Data::clear(const Sensor::Ptr & s)
	{
		//Clear ambiguous
		utility::safeAccess(ambiguous_measurements.sensors, s).clear();
	}
	
	void Correlator::Data::cleanUp()
	{
		//TODO: clean up unambiguous measurements no longer needed / which have been used
		//For now, just remove all measurements
		for(auto& node : unambiguous_measurements){
			for(auto& sensor : node.second.sensors){
				sensor.second.clear();
			}
		}
	}

	bool Correlator::Data::isCorrelated()
	{
		for (auto& s : ambiguous_measurements.sensors) {
			if (!s.first->isResolved()) return false;
		}
		return true;
	}

	/////////////////////////////////////////////////////////////////////////////////////////////////////////
	//									Correlator:Public
	/////////////////////////////////////////////////////////////////////////////////////////////////////////

	void Correlator::addMeasurementGroup(const std::vector<Measurement::Ptr>& measurementQueue) {
		//Check there is data corresponding to more than one system for a given node, otherwise useless
		auto measurements = filterLonelyData(measurementQueue);

		//Decide if data is useful
		//(if at least one stream has changed relative to previous measurements)
		bool dataNovel = checkChanges(measurements);

		if (dataNovel) {
			//Store the (refs to) the relevant measurements
			//FUSION_LOG("Adding calibration measurments!!");
			for (auto& m : measurements) {
				addMeasurement(m);
			}

		}
	}

	void Correlator::identify()
	{
		//Resolve ambiguities whenever data permits
		for(auto& pair : data.ambiguous_measurements.sensors){
			Sensor::Ptr sensor = pair.first;
			std::vector<Measurement::Ptr>& stream = pair.second;

			if(!dataSufficient(sensor)) continue;

			//get sensor node info
			const std::set<NodeDescriptor>& possible_nodes = sensor->getNodes();

			//Initialise scores:
			std::map<NodeDescriptor, float> score;

			//For each possible node
			for(auto& node : possible_nodes){
				const Data::Streams& unambiguousStreams = data.getUnambiguousStreams(node);

				if(!sensor->nodeEliminated(node)){
					score[node] = getCorrelationScore(stream, unambiguousStreams);
				} else {
					score[node] = 0;
				}

				if(score[node] < elimination_threshold){
					sensor->eliminateNode(node);
				}
			}

			//Clear data used
			data.clear(sensor);
		}
		//TODO: implement the following method:
		data.cleanUp();
	}

	bool Correlator::isStable()
	{
		return data.isCorrelated();
	}
	
	/////////////////////////////////////////////////////////////////////////////////////////////////////////
	//									Correlator:Private
	/////////////////////////////////////////////////////////////////////////////////////////////////////////
	void Correlator::addMeasurement(const Measurement::Ptr& m){
		if(m->isAmbiguous()){
			addAmbiguousMeasurement(m);
		} else {
			addUnambiguousMeasurementIfNeeded(m);
		}
	}

	void Correlator::addAmbiguousMeasurement(const Measurement::Ptr& m)
	{
		data.addAmbiguous(m);
	}

	void Correlator::addUnambiguousMeasurementIfNeeded(const Measurement::Ptr& m)
	{
		if (data.unambiguousMeasurementNeeded(m->getSensor())) {
			data.addUnambiguous(m);
		}
	}

	bool Correlator::dataSufficient(const Sensor::Ptr & sensor)
	{
		return data.ambiguousCount(sensor) > ambiguous_threshold;
	}


	std::vector<Measurement::Ptr>
	Correlator::filterLonelyData(const std::vector<Measurement::Ptr>& measurementQueue) {
		//Init result
		std::vector<Measurement::Ptr> result;
		//Structure for counting systems per node
		std::set<NodeDescriptor> nodesWithAmbiguousSensors;

		//Count
		for (auto& m : measurementQueue) {
			if(m->isAmbiguous()){
				for(auto& node : m->getNodes()){
					nodesWithAmbiguousSensors.insert(node);
				}	
			}
		}
		//Push back relevant measurments
		//TODO: throwout ambiguous with no matching unambiguous?
		for (auto& m : measurementQueue) {
			if (m->isAmbiguous() || //push back ambigous
			    nodesWithAmbiguousSensors.count(m->getNode()) > 0 //Push back unambiguous with useful info
			   ) 
			{
				result.push_back(m);
			}
		}
		return result;
	}

	bool Correlator::checkChanges(const std::vector<Measurement::Ptr>& measurements) {
		//Check change for each measurement
		bool result = false;
		for (auto& mes : measurements) {
			auto node = mes->getNode();
			float diff = data.compareMeasurement(mes);
			//TODO:Perform next check over each node individually
			//If any of the measurements are new then return true

			result = result || (diff > diff_threshold);
		}
		return result;
	}
}
