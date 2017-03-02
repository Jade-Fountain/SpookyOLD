#include "UnrealFusion.h"
#include "Core.h"
namespace fusion {

	void Core::addNode(const NodeDescriptor & node, const NodeDescriptor & parent)
	{
		skeleton.addNode(node, parent);
	}

	void Core::finaliseSetup()
	{
	}

	//Adds a new measurement to the system
	void Core::addMeasurement(const Measurement::Ptr& m, const NodeDescriptor& node) {
		//systems[m->system].addMeasurement(m);
		m->addNode(node);
		skeleton.addMeasurement(node, m);
	}

	//Computes data added since last fuse() call. Should be called repeatedly	
	void Core::fuse() {
		//Add new data to calibration, with checking for usefulness
		calibrator.addMeasurementGroup(skeleton.getMeasurements());
		calibrator.calibrate();
		skeleton.fuse();
	}

	CalibrationResult Core::getCalibrationResult(SystemDescriptor s1, SystemDescriptor s2) {
		return calibrator.getResultsFor(s1, s2);
	}

	void Core::setMeasurementSensorInfo(Measurement::Ptr & m, SystemDescriptor system, SensorID id)
	{
		//If we haven't seen this sensor already, initialise
		if (utility::safeAccess(sensors, system).count(id) == 0) {
			utility::safeAccess(sensors, system)[id] = std::make_unique<Sensor>();
			utility::safeAccess(sensors, system)[id]->system = system;
			utility::safeAccess(sensors, system)[id]->id = id;
		}
		//Set pointer in measurement
		m->setSensor(sensors[system][id]);
	}

}
