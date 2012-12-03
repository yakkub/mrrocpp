#if !defined(__MP_T_JK_TEST_H)
#define __MP_T_JK_TEST_H

#include "base/mp/mp_task.h"

namespace mrrocpp {
namespace mp {
namespace task {

/**
 * @defgroup swarm_demo swarm_demo
 * @ingroup application
 * A swarm demo application
 */

class mp_jk_test : public task
{
protected:

public:

	/**
	 * Constructor.
	 */
	mp_jk_test(lib::configurator &_config);
	/// utworzenie robotow
	void create_robots(void);
	void main_task_algorithm(void);

};

} // namespace task
} // namespace mp
} // namespace mrrocpp

#endif
