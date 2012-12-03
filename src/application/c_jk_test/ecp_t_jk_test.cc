#include <stdexcept>

#include "ecp_t_jk_test.h"
#include "generator/ecp/smooth_file_from_mp/ecp_g_smooth_file_from_mp.h"

// ecp_robots headers
#include "robot/irp6ot_m/ecp_r_irp6ot_m.h"
#include "robot/irp6p_m/ecp_r_irp6p_m.h"


using namespace std;

namespace mrrocpp {
namespace ecp {
namespace common {
namespace task {


// KONSTRUKTORY
jk_test::jk_test(mrrocpp::lib::configurator &_config) :
				common::task::task(_config)
{
	// the robot is choose dependently on the section of configuration file sent as argv[4]
	if (config.robot_name == lib::irp6ot_m::ROBOT_NAME) {
		ecp_m_robot = (boost::shared_ptr <robot_t>) new irp6ot_m::robot(*this);
	} else if (config.robot_name == lib::irp6p_m::ROBOT_NAME) {
		ecp_m_robot = (boost::shared_ptr <robot_t>) new irp6p_m::robot(*this);
	} else {
		throw std::runtime_error("Robot not supported");
	}

	register_generator(new generator::smooth_file_from_mp(*this, lib::ECP_JOINT, ecp_mp::generator::ECP_GEN_SMOOTH_JOINT_FILE_FROM_MP, true));
}

task_base* return_created_ecp_task(lib::configurator &_config)
{
	return new jk_test(_config);
}

} // namespace task
} // namespace common
} // namespace ecp
} // namespace mrrocpp
