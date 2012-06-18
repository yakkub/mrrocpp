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
	while (!boost::this_thread::interruption_requested()) {
		delay(10);
	//	sr_msg->message("imu operator() in while");
	}
	sr_msg->message("imu operator() interruption_requested");
}

imu::imu(common::manip_effector &_master) :
		master(_master)
{
	sr_msg =
			boost::shared_ptr <lib::sr_vsp>(new lib::sr_vsp(lib::EDP, "i_" + master.config.robot_name, master.config.get_sr_attach_point()));

	sr_msg->message("imu constructor");
}

imu::~imu()
{
	sr_msg->message("~imu destructor");
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
