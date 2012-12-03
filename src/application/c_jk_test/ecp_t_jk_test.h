#if !defined(_ECP_T_JK_TEST_H)
#define _ECP_T_JK_TEST_H

#include "base/ecp/ecp_task.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace task {

class jk_test: public common::task::task  {
	public:
		// KONSTRUKTORY
		jk_test(mrrocpp::lib::configurator &_config);

};

} // namespace task
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif
