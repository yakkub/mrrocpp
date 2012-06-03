#if !defined(_ECP_ST_ACQUISITION_H)
#define _ECP_ST_ACQUISITION_H

#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"
#include "base/ecp/ecp_subtask.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace subtask {

class acquisition : public subtask
{
public:
	// KONSTRUKTORY
	acquisition(task::task &_ecp_t);

	// methods for ECP template to redefine in concrete classes
	void main_task_algorithm(void);

	virtual void
			write_data(const std::string & _K_fp, const std::string & _kk_fp, const std::string & _M_fp, const std::string & _mm_fp, int _number_of_measures) = 0;
};

} // namespace subtask
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif
