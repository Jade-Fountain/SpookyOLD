
#include "UnrealFusion.h"
#include "Correlator.h"

namespace fusion {
	//---------------------------------------------------------------------------------
	//										Calibrator::Data
	//---------------------------------------------------------------------------------

	void Correlator::Data::addAmbiguous(const Sensor::Ptr& sensor, const Measurement::Ptr& m) {
		if (ambiguous_measurements.sensors.count(sensor) == 0) {
			ambiguous_measurements.sensors[sensor] = std::vector<Measurement::Ptr>();
			const std::set<NodeDescriptor>& nodes = sensor->getNodes();
			relevant_nodes.insert(nodes.begin(), nodes.end());
		} else {
			ambiguous_measurements.sensors[sensor].push_back(m);
		}
	}

	void Correlator::Data::addUnambiguous(const Sensor::Ptr& sensor, const Measurement::Ptr& m) {
		utility::safeAccess(
			utility::safeAccess(unambiguous_measurements, m->getNode()).sensors,
			sensor
			).push_back(m);
	}

	bool Correlator::Data::unambiguousMeasurementNeeded(const Sensor::Ptr& s) {
		return relevant_nodes.count(s->getNode()) > 0;
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
		
	}
	//---------------------------------------------------------------------------------
	//										Calibrator
	//---------------------------------------------------------------------------------

	void Correlator::addAmbiguousMeasurement(const Measurement::Ptr& m)
	{
		data.addAmbiguous(m->getSensor(), m);
	}

	void Correlator::addUnambiguousMeasurementIfNeeded(const Measurement::Ptr& m)
	{
		if (data.unambiguousMeasurementNeeded(m->getSensor())) {
			data.addUnambiguous(m->getSensor(), m);
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
	
	bool Correlator::dataSufficient(const Sensor::Ptr & sensor)
	{
		//TODO:: include unambiguous count
		return data.ambiguousCount(sensor) > ambiguous_threshold;
	}


}
