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
	if (!master.robot_test_mode) {
		lib::set_thread_priority(lib::PTHREAD_MAX_PRIORITY - 1);
	}

	try {
		if (!imu_sensor_test_mode) {
			connect_to_hardware();
		}

		thread_started.command();
		//	first_measure_synchroniser.command();
	}

	catch (lib::exception::se_sensor & error) {
		std::cerr << "sensor_error w force thread EDP" << std::endl;

		uint64_t error0 = 0;

		if (uint64_t const * tmp = boost::get_error_info <mrrocpp_error0>(error)) {
			error0 = *tmp;
		}

		switch (error0)
		{
			case SENSOR_NOT_CONFIGURED:
				//		from_vsp.vsp_report = lib::sensor::VSP_SENSOR_NOT_CONFIGURED;
				break;
			case READING_NOT_READY:
				//		from_vsp.vsp_report = lib::sensor::VSP_READING_NOT_READY;
				break;
		}
		sr_msg->message(lib::FATAL_ERROR, error0);

	}

	catch (std::exception & e) {
		printf("force sensor exception: %s\n", e.what());
		sr_msg->message(lib::FATAL_ERROR, e.what());
		exit(EXIT_SUCCESS);
	}

	catch (...) {
		std::cerr << "unidentified error force thread w EDP" << std::endl;
	}

	while (!boost::this_thread::interruption_requested()) {

		try {
			if (new_edp_command) {
				//sr_msg->message("new_edp_command");
				boost::mutex::scoped_lock lock(mtx);
				// TODO: this should be handled with boost::bind functor parameter
				switch (command)
				{
					case (common::IMU_CONFIGURE):
						configure_sensor();
						break;
					default:
						break;
				}
				set_command_execution_finish();
			} else {

				//sr_msg->message("else");
				wait_for_event();
				//	sr_msg->message("za wait_for_event");
				get_reading();
				//	sr_msg->message("za get_reading");
				first_measure_synchroniser.command();
			}

		} //!< koniec TRY

		catch (lib::exception::se_sensor & error) {
			std::cerr << "sensor_error w force thread EDP" << std::endl;

			uint64_t error0 = 0;

			if (uint64_t const * tmp = boost::get_error_info <mrrocpp_error0>(error)) {
				error0 = *tmp;
			}

			switch (error0)
			{
				case SENSOR_NOT_CONFIGURED:
					//	from_vsp.vsp_report = lib::sensor::VSP_SENSOR_NOT_CONFIGURED;
					break;
				case READING_NOT_READY:
					//	from_vsp.vsp_report = lib::sensor::VSP_READING_NOT_READY;
					break;
			}
			sr_msg->message(lib::FATAL_ERROR, error0);

		}

		catch (...) {
			std::cerr << "unidentified error in EDP force thread" << std::endl;
		}

		//	sr_msg->message("imu operator() in while");
	}
	sr_msg->message("imu operator() interruption_requested");
}

/**************************** inicjacja czujnika ****************************/
void imu::configure_sensor(void)
{ // by Y

	//  printf("edp Sensor configured\n");
	sr_msg->message("IMU sensor being configured");

	if (!imu_sensor_test_mode) {
		configure_particular_sensor();
	}

}

imu::imu(common::manip_effector &_master) :
		imu_sensor_test_mode(true), master(_master), new_edp_command(false)
{
	sr_msg =
			boost::shared_ptr <lib::sr_vsp>(new lib::sr_vsp(lib::EDP, "i_" + master.config.robot_name, master.config.get_sr_attach_point()));

	sr_msg->message("imu constructor");

	if (master.config.exists(lib::IMU_SENSOR_TEST_MODE)) {
		imu_sensor_test_mode = master.config.exists_and_true(lib::IMU_SENSOR_TEST_MODE);
	}

	if (imu_sensor_test_mode) {
		sr_msg->message("IMU sensor test mode activated");
	}

}

imu::~imu()
{
	sr_msg->message("~imu destructor");
}

void imu::wait_for_event()
{
	if (!imu_sensor_test_mode) {

		wait_for_particular_event();

	} else {
		usleep(1000);
	}
}

/***************************** odczyt z czujnika *****************************/
void imu::get_reading(void)
{
	if (!imu_sensor_test_mode) {
		get_particular_reading();

	} else {

	}

	if (master.rb_obj) {
		boost::mutex::scoped_lock lock(master.rb_obj->reader_mutex);
		master.rb_obj->step_data.real_cartesian_acc[0] = ldata.linearAcceleration[0];
		master.rb_obj->step_data.real_cartesian_acc[1] = ldata.linearAcceleration[1];
		master.rb_obj->step_data.real_cartesian_acc[2] = ldata.linearAcceleration[2];
		master.rb_obj->step_data.real_cartesian_vel[3] = ldata.angularVelocity[0];
		master.rb_obj->step_data.real_cartesian_vel[4] = ldata.angularVelocity[1];
		master.rb_obj->step_data.real_cartesian_vel[5] = ldata.angularVelocity[2];

	}

}

void imu::set_command_execution_finish() // podniesienie semafora
{
	new_edp_command = false;
	new_command_synchroniser.command();
}

} // namespace sensor
} // namespace edp
} // namespace mrrocpp
