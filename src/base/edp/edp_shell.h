/*!
 * \file edp_shell.h
 * \author yoyek
 * \date 2011
 *
 */

#ifndef __EDP_SHELL_H
#define __EDP_SHELL_H

#include <boost/shared_ptr.hpp>

#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"

#include "edp_typedefs.h"

#include "base/lib/sr/sr_edp.h"
#include "base/lib/configurator.h"

#include "edp_exceptions.h"

namespace mrrocpp {
namespace edp {
namespace common {

/*!
 * \class shell
 * \brief EDP shell class
 *
 * Used in edp master process
 *
 * \author yoyek
 */
class shell
{

	friend class effector;

private:

	/*!
	 * \brief Reference to configuration object
	 *
	 * It stores data read from ini file.
	 */
	lib::configurator &config;

	/*!
	 * \brief full path to the hardware busy file
	 *
	 */
	std::string hardware_busy_file_fullpath;

	/*!
	 * \brief EDP pid
	 *
	 */
	const pid_t my_pid;

	/*!
	 * \brief Method to close hardware busy notification file
	 *
	 */
	void close_hardware_busy_file(void);

public:
	/*!
	 * \brief Pointer to object to communicate with UI SR thread outside the signal handlers.
	 *
	 * For the usage in asynchronous communication.
	 */
	boost::shared_ptr <lib::sr_edp> msg;

	shell(lib::configurator &_config);

	//! Destructor
	~shell();
	bool detect_hardware_busy(void);

	/*!
	 * \brief Method to create hardware busy notification file
	 * it writes process pid into the file
	 *
	 */
	bool create_hardware_busy_file();

};

} // namespace common
} // namespace edp
} // namespace mrrocpp

#endif
