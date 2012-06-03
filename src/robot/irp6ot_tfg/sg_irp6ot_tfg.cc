/* --------------------------------------------------------------------- */
/*                          SERVO_GROUP Process                          */
// ostatnia modyfikacja - styczen 2005
/* --------------------------------------------------------------------- */

#include <cstdio>
#include <cstdlib>

#include <unistd.h>

// Klasa edp_irp6ot_effector.
#include "robot/irp6ot_tfg/edp_irp6ot_tfg_effector.h"
#include "base/edp/reader.h"
// Klasa hardware_interface.
#include "robot/hi_moxa/hi_moxa.h"
// Klasa servo_buffer.
#include "robot/irp6ot_tfg/sg_irp6ot_tfg.h"
#include "robot/irp6ot_tfg/regulator_irp6ot_tfg.h"

namespace mrrocpp {
namespace edp {
namespace irp6ot_tfg {

/*-----------------------------------------------------------------------*/
servo_buffer::servo_buffer(effector &_master) :
	common::servo_buffer(_master), master(_master)
{
	synchro_axis_order[0] = 0;
	axe_inc_per_revolution[0] = AXIS_7_INC_PER_REVOLUTION;
	synchro_step_coarse[0] = AXIS_7_SYNCHRO_STEP_COARSE;
	synchro_step_fine[0] = AXIS_7_SYNCHRO_STEP_FINE;

	thread_id = boost::thread(boost::bind(&servo_buffer::operator(), this));
}
/*-----------------------------------------------------------------------*/

void servo_buffer::load_hardware_interface(void)
{
	// tablica pradow maksymalnych dla poszczegolnych osi
		//int max_current[lib::irp6p_tfg::NUM_OF_SERVOS] = { AXIS_7_MAX_CURRENT };

		const std::vector <std::string>
				ports_vector(mrrocpp::lib::irp6ot_tfg::ports_strings, mrrocpp::lib::irp6ot_tfg::ports_strings
						+ mrrocpp::lib::irp6ot_tfg::LAST_MOXA_PORT_NUM + 1);
		hi
				= new hi_moxa::HI_moxa(master, mrrocpp::lib::irp6ot_tfg::LAST_MOXA_PORT_NUM, ports_vector, mrrocpp::lib::irp6ot_tfg::MAX_INCREMENT);
		hi->init();

		//Ustawienie zwlocznego ograniczenia pradowego - dlugotrwale przekroczenie ustawionej wartosci
		//spowoduje wlaczenie stopu awaryjnego przez sterownik
		hi->set_parameter(0, hi_moxa::PARAM_MAXCURRENT, mrrocpp::lib::irp6ot_tfg::MAX_CURRENT_0);

	// utworzenie tablicy regulatorow
	// Serwomechanizm 1
        regulator_ptr[0] = new NL_regulator_8_irp6ot(0, 0, 0, 0.39, 8.62 / 2., 7.89 / 2., 0.35, master);

	common::servo_buffer::load_hardware_interface();
}

/*-----------------------------------------------------------------------*/
void servo_buffer::get_all_positions(void)
{
	common::servo_buffer::get_all_positions();

	// przepisanie stanu regulatora chwytaka do bufora odpowiedzi dla EDP_master
	servo_data.gripper_reg_state = regulator_ptr[0]->get_reg_state();

}
/*-----------------------------------------------------------------------*/

} // namespace irp6ot
} // namespace edp
} // namespace mrrocpp
