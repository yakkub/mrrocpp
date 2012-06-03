#if !defined(_IRP6OT_TFG_CONST_H)
#define _IRP6OT_TFG_CONST_H

/*!
 * @file
 * @brief File contains constants and structures for IRp6 on track two finger gripper
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup irp6ot_tfg
 */

#include "base/lib/impconst.h"

namespace mrrocpp {
namespace lib {
namespace irp6ot_tfg {

/*!
 * @brief IRp6 on track two finger gripper robot label
 * @ingroup irp6ot_tfg
 */
const robot_name_t ROBOT_NAME = "irp6ot_tfg";

/*!
 * @brief IRp6 on track two finger gripper total number of servos
 * @ingroup irp6ot_tfg
 */
const int NUM_OF_SERVOS = 1;

/*!
 * @brief IRp6 on track two finger gripper last Moxa port number [0..7]
 * @ingroup sarkofag
 */
const int LAST_MOXA_PORT_NUM = 0;

/*!
 * @brief IRp6 on track two finger gripper array of communication port names
 * @ingroup sarkofag
 */

const std::string ports_strings[] = //{ "/dev/ttyMUE6" };
{ "/dev/ttyMI6" };

/*!
 * @brief IRp6 on track two finger gripper overcurrent threshold [mA]
 * @ingroup irp6p_tfg
 */
const int MAX_CURRENT_0 = 200;

/*!
 * @brief IRp6 on track two finger gripper overcurrent threshold [mA]
 * @ingroup irp6p_tfg
 */
const double MAX_INCREMENT[] = { 1000 };

}
} // namespace lib
} // namespace mrrocpp

#endif /* _IRP6OT_TFG_CONST_H */
