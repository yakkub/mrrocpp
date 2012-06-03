/*!
 * @file
 * @brief File containing methods of the base kinematic model for two fingered grippers.
 *
 * @author yoyek
 * @author tkornuta
 * @date Jun 21, 2010
 *
 * @ingroup KINEMATICS irp6_tfg
 */

#include <cmath>

// for MacOS compatibility, where isnan() is implemented as a function in the std:: namespace
// using std::isnan;

#include "base/lib/com_buf.h"
#include "robot/irp6_tfg/kinematic_model_irp6_tfg.h"

using namespace mrrocpp::lib::exception;

namespace mrrocpp {
namespace kinematics {
namespace irp6_tfg {

kinematic_model_irp6_tfg::kinematic_model_irp6_tfg(void)
{
}

void kinematic_model_irp6_tfg::check_motor_position(const lib::MotorArray & motor_position) const
{
	if (motor_position[0] < lower_limit_axis) {
		BOOST_THROW_EXCEPTION(nfe_2() << mrrocpp_error0(BEYOND_LOWER_LIMIT_AXIS_0));
	} else if (motor_position[0] > upper_limit_axis) {
		BOOST_THROW_EXCEPTION(nfe_2() << mrrocpp_error0(BEYOND_UPPER_LIMIT_AXIS_0));
	}

} //: check_motor_position

void kinematic_model_irp6_tfg::check_joints(const lib::JointArray & q) const
{
	if (isnan(q[0])) {
		BOOST_THROW_EXCEPTION(nfe_2() << mrrocpp_error0(NOT_A_NUMBER_JOINT_VALUE_THETA1));

	}

	if (q[0] < lower_limit_joint) {
		BOOST_THROW_EXCEPTION(nfe_2() << mrrocpp_error0(BEYOND_LOWER_THETA1_LIMIT));
	}

	if (q[0] > upper_limit_joint) {
		BOOST_THROW_EXCEPTION(nfe_2() << mrrocpp_error0(BEYOND_UPPER_THETA1_LIMIT));
	}

} //: check_joints

void kinematic_model_irp6_tfg::mp2i_transform(const lib::MotorArray & local_current_motor_pos, lib::JointArray & local_current_joints)
{
	local_current_joints[0] = dir_a_7 * (local_current_motor_pos[0] * local_current_motor_pos[0])
			- dir_b_7 * local_current_motor_pos[0] + dir_c_7;

	// Sprawdzenie obliczonych wartosci wspolrzednych wewnetrznych.
	check_joints(local_current_joints);

} //: mp2i_transform

void kinematic_model_irp6_tfg::i2mp_transform(lib::MotorArray & local_desired_motor_pos_new, const lib::JointArray & local_desired_joints)
{

	// Obliczenie kata obrotu walu silnika napedowego chwytaka.
	local_desired_motor_pos_new[0] = inv_a_7 * sqrt(inv_b_7 + inv_c_7 * local_desired_joints[0]) + inv_d_7;

	// Sprawdzenie obliczonych wartosci wspolrzednych wewnetrznych.
	check_joints(local_desired_joints);

	// Sprawdzenie obliczonych wartosci.
	check_motor_position(local_desired_motor_pos_new);

} //: i2mp_transform

} // namespace irp6_tfg
} // namespace kinematic
} // namespace mrrocpp

