#include "base/ecp/ecp_robot.h"
#include "ecp_g_smooth_file_from_mp.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

smooth_file_from_mp::smooth_file_from_mp(task::task & _ecp_t, lib::ECP_POSE_SPECIFICATION pose_spec, std::string _LABEL, bool _detect_jerks) :
		common::generator::generator(_ecp_t), detect_jerks(_detect_jerks)
{
	generator_name = _LABEL;
	switch (pose_spec)
	{
		case lib::ECP_JOINT:
			sgen =
					(boost::shared_ptr <newsmooth>) new newsmooth(_ecp_t, pose_spec, ecp_t.ecp_m_robot->number_of_servos);
			sgen->set_debug(true);
			break;
		case lib::ECP_XYZ_ANGLE_AXIS:
			sgen = (boost::shared_ptr <newsmooth>) new newsmooth(_ecp_t, pose_spec, 6);
			sgen->set_debug(true);
			break;
		default:
			break;
	}
}

void smooth_file_from_mp::conditional_execution()
{
	sgen->reset();

	ecp_t.mp_command.ecp_next_state.sg_buf.get(path);

	//path = ecp_t.mp_command.ecp_next_state.get_mp_2_ecp_next_state_string();
	sgen->load_trajectory_from_file(path.c_str());

	if (detect_jerks) {
                if (sgen->calculate_interpolate() /*&& sgen->detect_jerks(1) == 0*/) {
			sgen->Move();
		}
	} else {
		if (sgen->calculate_interpolate()) {
			sgen->Move();
		}
	}
}

} // namespace task
} // namespace common
} // namespace ecp
} // namespace mrrocpp
