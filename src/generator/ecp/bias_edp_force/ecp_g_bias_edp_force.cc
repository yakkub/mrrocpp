/*!
 * @file
 * @brief File contains bias_edp_force generator definition
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup generators
 */

#include "base/ecp/ecp_robot.h"
#include "ecp_g_bias_edp_force.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

// // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // ///////////////////
//
// 			bias_edp_force_generator
//
// // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // ///////////////////

bias_edp_force::bias_edp_force(common::task::task& _ecp_task) :
		common::generator::generator(_ecp_task)
{
	generator_name = ecp_mp::generator::ECP_GEN_BIAS_EDP_FORCE;
}

bool bias_edp_force::first_step()
{

//	std::cout << "bias_edp_force" << node_counter << std::endl;

	the_robot->ecp_command.instruction_type = lib::SET;
	the_robot->ecp_command.set_type = ROBOT_MODEL_DEFINITION;
	the_robot->ecp_command.robot_model.type = lib::FORCE_BIAS;

	return true;
}

} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp
