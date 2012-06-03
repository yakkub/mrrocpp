#if !defined(_IRP6P_M_CONST_H)
#define _IRP6P_M_CONST_H

/*!
 * @file
 * @brief File contains constants and structures for IRp6 postument manipulator
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup irp6p_m
 */

#include<string>

#include "base/lib/impconst.h"

namespace mrrocpp {
namespace lib {
namespace irp6p_m {

/*!
 * @brief IRp6 postument manipulator robot label
 * @ingroup irp6p_m
 */
const robot_name_t ROBOT_NAME = "irp6p_m";

/*!
 * @brief IRp6 postument manipulator total number of servos
 * @ingroup irp6p_m
 */
const int NUM_OF_SERVOS = 6;

/*!
 * @brief IRp6 postument last Moxa port number [0..7]
 * @ingroup irp6p_m
 */
const int LAST_MOXA_PORT_NUM = 5;

/*!
 * @brief IRp6 postument array of communication port names
 * @ingroup irp6p_m
 */

const std::string ports_strings[] =
//		{ "/dev/ttyM0", "/dev/ttyM1", "/dev/ttyM2", "/dev/ttyM3", "/dev/ttyM4", "/dev/ttyM5" };
		{ "/dev/ttyMI8", "/dev/ttyMI9", "/dev/ttyMI10", "/dev/ttyMI11", "/dev/ttyMI12", "/dev/ttyMI13" };

/*!
 * @brief IRp6 postument overcurrent threshold [mA]
 * @ingroup irp6p_m
 */
const int MAX_CURRENT_0 = 15000;
const int MAX_CURRENT_1 = 18000;
const int MAX_CURRENT_2 = 10000;
const int MAX_CURRENT_3 = 10000;
const int MAX_CURRENT_4 = 10000;
const int MAX_CURRENT_5 = 10000;

/*!
 * @brief IRp6 postument max encoder increment
 * @ingroup irp6p_m
 */
const double MAX_INCREMENT[] = { 1000, 1000, 1000, 1000, 1000, 1000 };

} // namespace irp6p_m
} // namespace lib
} // namespace mrrocpp

#endif /* _IRP6P_M_CONST_H */
