/* Y o u r   D e s c r i p t i o n                       */
/*                            AppBuilder Photon Code Lib */
/*                                         Version 2.01  */

#include "../irp6_m/wgt_irp6_m_joints.h"
#include "../irp6_m/wgt_irp6_m_motors.h"
#include "../irp6_m/wgt_irp6_m_euler.h"
#include "../irp6_m/wgt_irp6_m_angle_axis.h"
#include "../irp6_m/wgt_irp6_m_relative_angle_axis.h"
#include "../irp6_m/wgt_irp6_m_tool_angle_axis.h"
#include "../irp6_m/wgt_irp6_m_tool_euler.h"

#include "ui_r_irp6p_m.h"
#include "ui_ecp_r_irp6p_m.h"
#include "robot/irp6p_m/const_irp6p_m.h"
#include "../base/interface.h"

#include "../base/mainwindow.h"
#include "ui_mainwindow.h"

#include "../base/menu_bar.h"
#include "../base/menu_bar_action.h"
#include "../base/mp.h"
#include "../base/ui_robot.h"
#include <boost/foreach.hpp>

namespace mrrocpp {
namespace ui {
namespace irp6p_m {

//
//
// KLASA UiRobot
//
//

void UiRobot::open_c_motor_window()
{
	wgts[WGT_MOTORS]->my_open();
}

void UiRobot::open_c_joint_window()
{
	wgts[WGT_JOINTS]->my_open();
}



void UiRobot::create_ui_ecp_robot()
{
	common::UiRobot::ui_ecp_robot = ui_ecp_robot = new ui::irp6p_m::EcpRobot(*this);
}

void UiRobot::edp_create_int_extra_operations()
{
	wgts[WGT_MOTORS]->synchro_depended_init();
}

void UiRobot::move_to_synchro_position()
{

	for (int i = 0; i < number_of_servos; i++) {
		desired_pos[i] = 0.0;
	}
	eb.command(boost::bind(&ui::irp6p_m::UiRobot::execute_motor_motion, &(*this)));

}

void UiRobot::move_to_front_position()
{

	for (int i = 0; i < number_of_servos; i++) {
		desired_pos[i] = state.edp.front_position[i];
	}
	eb.command(boost::bind(&ui::irp6p_m::UiRobot::execute_joint_motion, &(*this)));

}

void UiRobot::move_to_preset_position(int variant)
{

	for (int i = 0; i < number_of_servos; i++) {
		desired_pos[i] = state.edp.preset_position[variant][i];
	}
	eb.command(boost::bind(&ui::irp6p_m::UiRobot::execute_joint_motion, &(*this)));

}

void UiRobot::synchronise()
{
	eb.command(boost::bind(&ui::irp6p_m::UiRobot::synchronise_int, &(*this)));
}

UiRobot::UiRobot(common::Interface& _interface) :
		irp6_m::UiRobot(_interface, lib::irp6p_m::ROBOT_NAME, lib::irp6p_m::NUM_OF_SERVOS)
{
	add_wgt <wgt_irp6_m_joints>(WGT_JOINTS, "Irp6p_m joints");
	add_wgt <wgt_irp6_m_motors>(WGT_MOTORS, "Irp6p_m motors");
	add_wgt <wgt_irp6_m_angle_axis>(WGT_ANGLE_AXIS, "Irp6p_m angle axis");
	add_wgt <wgt_irp6_m_euler>(WGT_EULER, "Irp6p_m euler");
	add_wgt <wgt_irp6_m_relative_angle_axis>(WGT_RELATIVE_ANGLE_AXIS, "Irp6p_m relative angle axis");
	add_wgt <wgt_irp6_m_tool_angle_axis>(WGT_TOOL_ANGLE_AXIS, "Irp6p_m tool angle axis");
	add_wgt <wgt_irp6_m_tool_euler>(WGT_TOOL_EULER, "Irp6p_m tool euler");
}

void UiRobot::setup_menubar()
{
	irp6_m::UiRobot::setup_menubar();

	robot_menu->setTitle(QApplication::translate("MainWindow", "Irp6&p_m", 0, QApplication::UnicodeUTF8));

}

}
} //namespace ui
} //namespace mrrocpp
