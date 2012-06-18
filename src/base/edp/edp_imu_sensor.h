// -------------------------------------------------------------------------
//
// Definicje klasy edp_force_sensor
//
// Autor: yoyek
// -------------------------------------------------------------------------

#if !defined(_EDP_IMU_SENSOR_H)
#define _EDP_IMU_SENSOR_H

#include <boost/utility.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/circular_buffer.hpp>
#include <Eigen/Core>

#include "base/lib/mrmath/ForceTrans.h"
#include "base/lib/sensor_interface.h"				// klasa bazowa sensor
#include "base/edp/edp_typedefs.h"				// klasa bazowa sensor
#include "base/lib/condition_synchroniser.h"
#include "base/lib/sr/sr_vsp.h"

namespace mrrocpp {
namespace edp {
namespace common {
class manip_effector;


}

namespace sensor {


/********** klasa czujnikow po stronie EDP **************/
class imu : public lib::sensor::sensor_interface
{
protected:
	common::manip_effector &master;

	void get_reading(void);

public:
	//! komunikacja z SR
	boost::shared_ptr <lib::sr_vsp> sr_msg;

	void operator()();

	imu(common::manip_effector &_master);

	virtual ~imu();

};
// end: class edp_force_sensor

// Zwrocenie stworzonego obiektu - czujnika. Funkcja implementowana w plikach klas dziedziczacych.
imu* return_created_edp_imu_sensor(common::manip_effector &_master);

} // namespace sensor
} // namespace edp
} // namespace mrrocpp

#endif
