#include "base/lib/mrmath/mrmath.h"

int debugi = 1;

namespace mrrocpp {
namespace lib {

/*
 ForceTrans::ForceTrans(const lib::Homog_matrix & init_frame, const lib::Homog_matrix & s_frame)
 {
 initialized = false;
 sensor_frame = s_frame;
 synchro(init_frame);
 double arm[3] = { X_AXIS_ARM , Y_AXIS_ARM , Z_AXIS_ARM };
 lib::K_vector point_of_gravity(arm);
 double weight = Z_AXIS_GRAVITY_FORCE;
 defineTool(weight, point_of_gravity);
 initialized = true;
 }
 */

ForceTrans::ForceTrans(const short l_force_sensor_name, const lib::Homog_matrix & init_frame, const lib::Homog_matrix & s_frame, const double weight, const lib::K_vector & point_of_gravity, bool _is_right_turn_frame) :
		force_sensor_name(l_force_sensor_name), initialized(false), is_right_turn_frame(true)
{

	is_right_turn_frame = _is_right_turn_frame;
	sensor_frame = s_frame;
	//	sensor_frame_translation = lib::Homog_matrix (sensor_frame.return_with_with_removed_rotation());
	// sensor_frame_translation.remove_rotation();
	//	sensor_frame_rotation  = lib::Homog_matrix (sensor_frame.return_with_with_removed_translation());
	// sensor_frame_rotation.remove_translation();
	// cout << sensor_frame;

	ft_tr_sensor_in_wrist = lib::Ft_tr(sensor_frame);

	// ft_tr_inv_sensor_translation_matrix = !ft_tr_sensor_translation_matrix;
	//	ft_tr_sensor_rotation_matrix = lib::Ft_v_tr (sensor_frame_rotation, lib::Ft_v_tr::FT);;
	// ft_tr_inv_sensor_rotation_matrix = !ft_tr_sensor_rotation_matrix;

	tool_weight = weight;
	gravity_arm_in_wrist = point_of_gravity;

	synchro(init_frame);
	defineTool(init_frame, weight, point_of_gravity);
	initialized = true;
}

void ForceTrans::defineTool(const lib::Homog_matrix & init_frame, const double weight, const lib::K_vector & point_of_gravity)
{
	tool_weight = weight;
	gravity_arm_in_wrist = point_of_gravity;
	//	gravity_force_in_base = lib::K_vector (0.0, 0.0, tool_weight);
	gravity_force_torque_in_base = lib::Ft_vector(0.0, 0.0, -tool_weight, 0.0, 0.0, 0.0);

	//	lib::frame_tab sens_rot = {{0,-1,0},{1,0,0},{0,0,1},{0,0,0}};
	//	lib::Homog_matrix sensor_rotation = lib::Homog_matrix(sens_rot);
	// orientacja koncowki manipulatora bez narzedzia
	lib::Homog_matrix current_orientation(init_frame.return_with_with_removed_translation());
	// cout << current_orientation << endl;
	// current_orientation.remove_translation(); // elminacja skladowej polozenia
	//	cout <<"aaaa"<<	endl << sensor_frame <<endl << sensor_rotation<<endl ;
	//	lib::K_vector gravity_force_in_sensor = (!(orientation*sensor_rotation))*gravity_force_in_base;
	// wyznaczenie sily grawitacji i z jej pomoca sil i momentow
	//	 lib::K_vector gravity_force_in_sensor = (!current_orientation)*gravity_force_in_base;
	lib::Ft_vector gravity_force_torque_in_sensor(lib::Ft_tr(!current_orientation) * gravity_force_torque_in_base);
	//	cout << orientation << endl;
	// wzynaczenie macierzy transformacji sil dla danego polozenia srodka ciezkosci narzedzia wzgledem czujnika
	lib::Homog_matrix tool_mass_center_translation(point_of_gravity[0], point_of_gravity[1], point_of_gravity[2]);
	ft_tool_mass_center_translation = lib::Ft_tr(tool_mass_center_translation);
	//	cout << tool_mass_center_translation << endl;

	//	reaction_torque_in_sensor = lib::K_vector((gravity_force_in_sensor*gravity_arm_in_sensor)*(-1));
	//	reaction_force_in_sensor = lib::K_vector(gravity_force_in_sensor*(-1));

	// reaction_torque_in_sensor = lib::K_vector((gravity_force_torque_in_sensor.get_force_lib::K_vector()*gravity_arm_in_sensor)*(-1));
	// reaction_force_in_sensor = lib::K_vector(gravity_force_torque_in_sensor.get_force_lib::K_vector()*(-1));

	//	cout << "aa:" << reaction_force_in_sensor << reaction_torque_in_sensor << endl;
	// wyznaczenie sil reakcji
	reaction_force_torque_in_sensor = -(ft_tool_mass_center_translation * gravity_force_torque_in_sensor);

	//	reaction_force_in_sensor = reaction_force_torque_in_sensor.get_force_lib::K_vector();
	//	reaction_torque_in_sensor = reaction_force_torque_in_sensor.get_torque_lib::K_vector();

	//	cout << "bb:" << reaction_force_torque_in_sensor << endl;
}

// zwraca sily i momenty sil w w ukladzie z orientacja koncowki manipulatory bez narzedzia
lib::Ft_vector ForceTrans::getForce(const lib::Ft_vector _inputForceTorque, const lib::Homog_matrix current_rotation)
{

	lib::Ft_vector inputForceTorque = _inputForceTorque;

	if (!is_right_turn_frame) {

		inputForceTorque[2] = -inputForceTorque[2];
		inputForceTorque[5] = -inputForceTorque[5];
	}

	if (initialized) {

		// sprowadzenie wejsciowych, zmierzonych sil i momentow sil z ukladu czujnika do ukladu nadgarstka
		lib::Ft_vector input_force_torque(ft_tr_sensor_in_wrist * inputForceTorque);

		// sprowadzenie odczytow sil do ukladu czujnika przy zalozeniu ze uklad chwytaka ma te sama orientacje
		// co uklad narzedzia
		lib::Ft_vector gravity_force_torque_in_sensor(lib::Ft_tr(!current_rotation) * gravity_force_torque_in_base);

		// finalne przeksztalcenie (3.30 z doktoratu TW)
		lib::Ft_vector output_force_torque(input_force_torque
				- (ft_tool_mass_center_translation * gravity_force_torque_in_sensor) - reaction_force_torque_in_sensor);

		// sprowadzenie sily w ukladzie nadgarstka do orientacji ukladu bazowego
		output_force_torque = lib::Ft_tr(current_rotation) * Ft_vector(-output_force_torque);

		return output_force_torque;
	}
	return 0;
}

void ForceTrans::synchro(const lib::Homog_matrix & init_frame)
{
	//initialisation_frame = init_frame;
	if (initialized)
		defineTool(init_frame, tool_weight, gravity_arm_in_wrist);
}

} // namespace lib
} // namespace mrrocpp
