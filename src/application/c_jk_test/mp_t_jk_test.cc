#include "mp_t_jk_test.h"

// ecp generators to be commanded
#include "generator/ecp/smooth_file_from_mp/ecp_mp_g_smooth_file_from_mp.h"

// mp_robots headers
#include "robot/irp6ot_m/mp_r_irp6ot_m.h"
#include "robot/irp6p_m/mp_r_irp6p_m.h"

namespace mrrocpp {
namespace mp {
namespace task {

// powolanie robotow w zaleznosci od zawartosci pliku konfiguracyjnego
void mp_jk_test::create_robots()
{
	ACTIVATE_MP_ROBOT(irp6ot_m);
//	ACTIVATE_MP_ROBOT(irp6p_m);
}

mp_jk_test::mp_jk_test(lib::configurator &_config) :
		task(_config)
{
}

void mp_jk_test::main_task_algorithm(void)
{
	sr_ecp_msg->message("JK test START");

	sr_ecp_msg->message("Joint");

	set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_JOINT_FILE_FROM_MP, 0, "../../src/application/c_jk_test/trj/start1_track.trj", lib::irp6ot_m::ROBOT_NAME);
	wait_for_task_termination(false, lib::irp6ot_m::ROBOT_NAME);

	sr_ecp_msg->message("JK demo END");

}

task* return_created_mp_task(lib::configurator &_config)
{
	return new mp_jk_test(_config);
}

} // namespace task
} // namespace mp
} // namespace mrrocpp
