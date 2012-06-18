#include <iostream>
#include <exception>

#include "edp_typedefs.h"
#include "edp_e_manip.h"
#include "base/lib/mis_fun.h"
#include "reader.h"
#include "base/kinematics/kinematic_model_with_tool.h"
#include "edp_imu_sensor.h"

namespace mrrocpp {
namespace edp {
namespace sensor {

void imu::operator()()
{
	while (1) {
		sleep(10);
	}
}

imu::imu(common::manip_effector &_master) :
		master(_master)
{

}

imu::~imu()
{
}

/***************************** odczyt z czujnika *****************************/
void imu::get_reading(void)
{

}

imu* return_created_edp_imu_sensor(common::manip_effector &_master)
{
	return new imu(_master);
}

} // namespace sensor
} // namespace edp
} // namespace mrrocpp
