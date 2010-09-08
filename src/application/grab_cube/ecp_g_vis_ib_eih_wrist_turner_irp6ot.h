/*
 * generator/ecp_g_vis_ib_eih_wrist_turner_irp6ot.h
 *
 *  Created on: DEC 10, 2009
 *      Author: rtulwin
 */

#ifndef ECP_VIS_IB_EIH_WRIST_TURNER_IRP6OT_H_
#define ECP_VIS_IB_EIH_WRIST_TURNER_IRP6OT_H_

#include "generator/ecp/ecp_g_visual_servo.h"

#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"

#include "sensor/fradia/ecp_mp_s_fradia_sensor.h"
#include "sensor/fradia/object_tracker.h"

namespace mrrocpp {
namespace ecp {
namespace irp6ot_m {
namespace generator {

#define MOTION_STEPS 25


class ecp_vis_ib_eih_wrist_turner_irp6ot: public common::generator::ecp_visual_servo {

public:

	ecp_mp::sensor::fradia_sensor<lib::empty_t, ecp_mp::sensor::object_tracker_t> *vsp_fradia; //wirtualny sensor
    double next_position[lib::MAX_SERVOS_NR]; 	//pozycja w nastepnym kroku.
    bool tracking; //jesli true, obiekt jest sledzony, jesli false, zagubiony (przychodzi z fradii)
    bool reached; //jesli true, pozycja nadgarstka jest odpowiednia do chwycenia (przychodzi z fradii)

	ecp_vis_ib_eih_wrist_turner_irp6ot(common::task::task& _ecp_task);
	virtual bool first_step(void);
	virtual bool next_step_without_constraints();
	virtual void limit_step();
};

} // namespace generator
} // namespace irp6ot
} // namespace ecp
} // namespace mrrocpp

#endif /* ECP_VIS_IB_EIH_WRIST_TURNER_IRP6OT_H_ */
