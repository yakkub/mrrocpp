// -------------------------------------------------------------------------
//                              task/mp_t_haptic.cc
//
// MP task for two robot haptic device
//
// -------------------------------------------------------------------------

#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"

#include "base/lib/sr/srlib.h"

#include "base/mp/mp_robot.h"
#include "base/mp/mp_task.h"
#include "application/haptic/mp_g_haptic.h"

#include "application/haptic/mp_t_haptic.h"
#include "application/rcsc/ecp_mp_t_rcsc.h"
#include "generator/ecp/bias_edp_force/ecp_mp_g_bias_edp_force.h"
#include "generator/ecp/transparent/ecp_mp_g_transparent.h"

#include "robot/irp6ot_m/mp_r_irp6ot_m.h"
#include "robot/irp6p_m/mp_r_irp6p_m.h"

namespace mrrocpp {
namespace mp {
namespace task {

task* return_created_mp_task(lib::configurator &_config)
{
	return new haptic(_config);
}

haptic::haptic(lib::configurator &_config) :
		task(_config)
{

}

// powolanie robotow w zaleznosci od zawartosci pliku konfiguracyjnego
void haptic::create_robots()
{

	ACTIVATE_MP_ROBOT(irp6ot_m);
	ACTIVATE_MP_ROBOT(irp6p_m);

}

void haptic::configure_edp_force_sensor(bool configure_track, bool configure_postument)
{
	if (configure_track) {
		set_next_ecp_state(ecp_mp::generator::ECP_GEN_BIAS_EDP_FORCE, 0, "", lib::irp6ot_m::ROBOT_NAME);
	}

	if (configure_postument) {
		set_next_ecp_state(ecp_mp::generator::ECP_GEN_BIAS_EDP_FORCE, 0, "", lib::irp6p_m::ROBOT_NAME);
	}

	if ((configure_track) && (!configure_postument)) {
		wait_for_task_termination(false, lib::irp6ot_m::ROBOT_NAME);
	} else if ((!configure_track) && (configure_postument)) {
		wait_for_task_termination(false, lib::irp6p_m::ROBOT_NAME);
	} else if ((configure_track) && (configure_postument)) {
		wait_for_task_termination(false, lib::irp6ot_m::ROBOT_NAME, lib::irp6p_m::ROBOT_NAME);
	}
}

void haptic::main_task_algorithm(void)
{
	generator::haptic mp_h_gen(*this, 4);
	mp_h_gen.robot_m = robot_m;

	sr_ecp_msg->message("New series");

	//pierwsza konfiguracja czujnikow
	// wlaczenie generatora do konfiguracji czujnika w EDP w obydwu robotach
	configure_edp_force_sensor(true, true);
	//	delay(1000);
	//	sr_ecp_msg->message("set_next_ecp_state");

	// wlaczenie generatora transparentnego w obu robotach
	set_next_ecp_state(ecp_mp::generator::ECP_GEN_TRANSPARENT, (int) 0, "", lib::irp6ot_m::ROBOT_NAME);
	set_next_ecp_state(ecp_mp::generator::ECP_GEN_TRANSPARENT, (int) 0, "", lib::irp6p_m::ROBOT_NAME);

	// mp_h_gen.sensor_m = sensor_m;

	sr_ecp_msg->message("Track podatny do czasu wcisniecia mp_trigger");
	mp_h_gen.Move();

	send_end_motion_to_ecps(lib::irp6ot_m::ROBOT_NAME, lib::irp6p_m::ROBOT_NAME);

	sr_ecp_msg->message("MP HAPTIC END");
}

} // namespace task
} // namespace mp
} // namespace mrrocpp
