
#include "UnrealFusion.h"
#include "Correlator.h"

namespace fusion {
	//---------------------------------------------------------------------------------
	//										Calibrator::Data
	//---------------------------------------------------------------------------------

	void Correlator::Data::addAmbiguous(const Sensor::Ptr& sensor, const Measurement::Ptr& m) {
		if (ambiguous_measurements.count(sensor) == 0) {
			ambiguous_measurements[sensor] = std::vector<Measurement::Ptr>();
			const std::set<NodeDescriptor>& nodes = sensor.getNodes();
			relevant_nodes.insert(nodes.begin(), nodes.end());
		} else {
			ambiguous_measurements[sensor].push_back(m);
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
		if(!data.sufficient()) return;
		//TODO: write code to resolve ambiguities whenever data permits
		for(auto& pair : ambiguous_measurements.sensors){
			Sensor::Ptr& sensor = pair.first;
			std::vector<Measurement::Ptr>& stream = pair.second;

			//get sensor node info
			std::set<NodeDescriptor>& possible_nodes = sensor->getNodes();

			//Initialise scores:
			std::map<NodeDescriptor, float>& scores;
			// float max_score = 0;
			// NodeDescriptor best_option;

			//For each possible node
			for(auto& node : possible_nodes){
				Data::Streams& unambiguousStreams = data.getStreams(node);
				//TODO: sync streams
				if(!sensor->nodeEliminated(node)){
					score[node] = getCorrelationScore(stream, unambiguousStreams);
				} else {
					score[node] = 0;
				}

				//TODO: eliminate or pick best or something

			}
		}
		data.clear();
	}

}
