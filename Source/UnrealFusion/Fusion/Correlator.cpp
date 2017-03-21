
#include "UnrealFusion.h"
#include "Correlator.h"

namespace fusion {

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

	}

}
