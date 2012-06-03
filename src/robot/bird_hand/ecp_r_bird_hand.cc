/*!
 * @file
 * @brief File contains ecp robot class definition for Bird Hand three finger gripper
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup bird_hand
 */

#include "robot/bird_hand/ecp_r_bird_hand.h"

namespace mrrocpp {
namespace ecp {
namespace bird_hand {

robot::robot(lib::configurator &_config, lib::sr_ecp &_sr_ecp) :
		ecp::common::robot::_ecp_robot <lib::bird_hand::c_buffer, lib::bird_hand::r_buffer>(lib::bird_hand::ROBOT_NAME, lib::bird_hand::NUM_OF_SERVOS, _config, _sr_ecp)
		,
		bird_hand_command_data_port(lib::bird_hand::COMMAND_DATA_PORT, port_manager)
		,
		bird_hand_configuration_command_data_port(lib::bird_hand::CONFIGURATION_DATA_PORT, port_manager)
		,
		bird_hand_status_reply_data_request_port(lib::bird_hand::STATUS_DATA_REQUEST_PORT, port_manager)
		,
		bird_hand_configuration_reply_data_request_port(lib::bird_hand::CONFIGURATION_DATA_REQUEST_PORT, port_manager)
{
	//  Stworzenie listy dostepnych kinematyk.
	create_kinematic_models_for_given_robot();
	data_ports_used = true;
}

robot::robot(common::task::task_base& _ecp_object) :
		ecp::common::robot::_ecp_robot <lib::bird_hand::c_buffer, lib::bird_hand::r_buffer>(lib::bird_hand::ROBOT_NAME, lib::bird_hand::NUM_OF_SERVOS, _ecp_object)
		,
		bird_hand_command_data_port(lib::bird_hand::COMMAND_DATA_PORT, port_manager)
		,
		bird_hand_configuration_command_data_port(lib::bird_hand::CONFIGURATION_DATA_PORT, port_manager)
		,
		bird_hand_status_reply_data_request_port(lib::bird_hand::STATUS_DATA_REQUEST_PORT, port_manager)
		,
		bird_hand_configuration_reply_data_request_port(lib::bird_hand::CONFIGURATION_DATA_REQUEST_PORT, port_manager)
{
	//  Stworzenie listy dostepnych kinematyk.
	create_kinematic_models_for_given_robot();
	data_ports_used = true;
}

void robot::create_command()
{
	// NOWE PORTY
	ecp_command.get_type = NOTHING_DEFINITION;

	if (bird_hand_status_reply_data_request_port.is_new_request()) {
		ecp_command.get_type |= ARM_DEFINITION;
		is_new_request = true;
	}

	if (bird_hand_configuration_reply_data_request_port.is_new_request()) {
		ecp_command.get_type |= ROBOT_MODEL_DEFINITION;
		is_new_request = true;
	}

	ecp_command.set_type = NOTHING_DEFINITION;

	if (bird_hand_command_data_port.get() == mrrocpp::lib::single_thread_port_interface::NewData) {
		ecp_command.set_type |= ARM_DEFINITION;

		ecp_command.bird_hand.command_structure.motion_steps = bird_hand_command_data_port.data.motion_steps;
		ecp_command.bird_hand.command_structure.ecp_query_step = bird_hand_command_data_port.data.ecp_query_step;

		ecp_command.bird_hand.command_structure.finger[0] = bird_hand_command_data_port.data.ring_f[0];
		ecp_command.bird_hand.command_structure.finger[1] = bird_hand_command_data_port.data.index_f[0];
		ecp_command.bird_hand.command_structure.finger[2] = bird_hand_command_data_port.data.index_f[1];
		ecp_command.bird_hand.command_structure.finger[3] = bird_hand_command_data_port.data.index_f[2];
		ecp_command.bird_hand.command_structure.finger[4] = bird_hand_command_data_port.data.thumb_f[1];
		ecp_command.bird_hand.command_structure.finger[5] = bird_hand_command_data_port.data.thumb_f[0];
		ecp_command.bird_hand.command_structure.finger[6] = bird_hand_command_data_port.data.ring_f[2];
		ecp_command.bird_hand.command_structure.finger[7] = bird_hand_command_data_port.data.ring_f[1];

		is_new_data = true;
	}

	if (bird_hand_configuration_command_data_port.get() == mrrocpp::lib::single_thread_port_interface::NewData) {
		ecp_command.set_type |= ROBOT_MODEL_DEFINITION;

		for (int i = 0; i < lib::bird_hand::THUMB_F_NUM_OF_SERVOS; i++) {
			ecp_command.bird_hand.configuration_command_structure.finger[i] =
					bird_hand_configuration_command_data_port.data.thumb_f[i];
		}

		for (int i = 0; i < lib::bird_hand::INDEX_F_NUM_OF_SERVOS; i++) {
			ecp_command.bird_hand.configuration_command_structure.finger[i + lib::bird_hand::THUMB_F_NUM_OF_SERVOS] =
					bird_hand_configuration_command_data_port.data.index_f[i];
		}

		for (int i = 0; i < lib::bird_hand::RING_F_NUM_OF_SERVOS; i++) {
			ecp_command.bird_hand.configuration_command_structure.finger[i + lib::bird_hand::THUMB_F_NUM_OF_SERVOS
					+ lib::bird_hand::RING_F_NUM_OF_SERVOS] = bird_hand_configuration_command_data_port.data.ring_f[i];
		}

		is_new_data = true;
	}

}

void robot::get_reply()
{

	// generator reply generation
	if (bird_hand_status_reply_data_request_port.is_new_request()) {

		bird_hand_status_reply_data_request_port.data.index_f[0] =
				reply_package.bird_hand.status_reply_structure.finger[1];
		bird_hand_status_reply_data_request_port.data.index_f[2] =
				reply_package.bird_hand.status_reply_structure.finger[3];
		bird_hand_status_reply_data_request_port.data.thumb_f[0] =
				reply_package.bird_hand.status_reply_structure.finger[5];
		bird_hand_status_reply_data_request_port.data.thumb_f[1] =
				reply_package.bird_hand.status_reply_structure.finger[4];
		bird_hand_status_reply_data_request_port.data.ring_f[1] =
				reply_package.bird_hand.status_reply_structure.finger[7];
		bird_hand_status_reply_data_request_port.data.ring_f[2] =
				reply_package.bird_hand.status_reply_structure.finger[6];
		bird_hand_status_reply_data_request_port.data.index_f[1] =
				reply_package.bird_hand.status_reply_structure.finger[2];
		bird_hand_status_reply_data_request_port.data.ring_f[0] =
				reply_package.bird_hand.status_reply_structure.finger[0];

		bird_hand_status_reply_data_request_port.set();
	}

	if (bird_hand_configuration_reply_data_request_port.is_new_request()) {

		for (int i = 0; i < lib::bird_hand::THUMB_F_NUM_OF_SERVOS; i++) {
			bird_hand_configuration_reply_data_request_port.data.thumb_f[i] =
					reply_package.bird_hand.configuration_reply_structure.finger[i];
		}

		for (int i = 0; i < lib::bird_hand::INDEX_F_NUM_OF_SERVOS; i++) {

			bird_hand_configuration_reply_data_request_port.data.index_f[i] =
					reply_package.bird_hand.configuration_reply_structure.finger[i
							+ lib::bird_hand::THUMB_F_NUM_OF_SERVOS];

		}

		for (int i = 0; i < lib::bird_hand::RING_F_NUM_OF_SERVOS; i++) {
			bird_hand_configuration_reply_data_request_port.data.ring_f[i] =
					reply_package.bird_hand.configuration_reply_structure.finger[i
							+ lib::bird_hand::THUMB_F_NUM_OF_SERVOS + lib::bird_hand::RING_F_NUM_OF_SERVOS];

		}

		bird_hand_configuration_reply_data_request_port.set();
	}

}

// Stworzenie modeli kinematyki dla robota IRp-6 na postumencie.
void robot::create_kinematic_models_for_given_robot(void)
{
	// Stworzenie wszystkich modeli kinematyki.
	add_kinematic_model(new kinematics::bird_hand::kinematic_model_bird_hand());
	// Ustawienie aktywnego modelu.
	set_kinematic_model(0);
}

} // namespace bird_hand
} // namespace ecp
} // namespace mrrocpp

