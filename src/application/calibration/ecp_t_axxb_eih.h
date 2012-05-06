#if !defined(_ECP_T_AXXB_EIH_H)
#define _ECP_T_AXXB_EIH_H

#include "ecp_t_calib_axxb.h"
#include "sensor/discode/discode_sensor.h"
#include <boost/shared_ptr.hpp>

namespace mrrocpp {

namespace lib {
	class configurator;
};

namespace ecp {
namespace common {
namespace task {

using namespace mrrocpp::ecp_mp::sensor::discode;

class axxb_eih: public calib_axxb  {
	public:
		// KONSTRUKTORY
		axxb_eih(mrrocpp::lib::configurator &_config);

		// methods for ECP template to redefine in concrete classes
		void main_task_algorithm(void);
	protected:
		boost::shared_ptr <discode_sensor> sensor;
};

} // namespace task
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif
