/// \file task/ecp_t_eih_acquisition.h
/// \brief definicja zadania akwizycji danych potrzebnych do kalibracji ukladu oko - reka
/// \author 2009 Jakub Kosiorek
///////////////////////////////////////////////////////////////////////////////

#if !defined(_ECP_ST_ACQ_EIH_H)
#define _ECP_ST_ACQ_EIH_H

#include <string>

#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"

#include <boost/shared_ptr.hpp>
#include "sensor/discode/discode_sensor.h"
#include "EIHReading.h"

#include "robot/irp6ot_m/ecp_r_irp6ot_m.h"
#include "robot/irp6p_m/ecp_r_irp6p_m.h"
#include "base/ecp/ecp_sub_task.h"
#include "generator/ecp/ecp_g_newsmooth.h"
#include "base/ecp/ecp_g_transparent.h"

#include "ecp_g_eih_nose_run.h"
#include "ecp_st_acquisition.h"
#include "ecp_g_eihcalibration.h"
#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>

#define POSTUMENT 0
#define TRACK 1

namespace mrrocpp {
namespace ecp {
namespace common {
namespace sub_task {

using namespace mrrocpp::ecp_mp::sensor::discode;

class acq_eih : public acquisition
{
private:
	std::string smooth_path;
	int delay_ms, robot, M;
	double A, C, D, E, vel, acc;
	bool calibrated;
	struct objective_function_parameters
	{
		// rotation matrix (from robot base to tool frame) - received from MRROC
		gsl_matrix *K;
		// rotation matrix (from chessboard base to camera frame)
		gsl_matrix *M;
		// translation vector (from robot base to tool frame) - received from MRROC
		gsl_vector *k;
		// translation vector (from chessboard base to camera frame)
		gsl_vector *m;
		// how many measurements were taken
		int number_of_measures;
	} ofp;

	//ecp_mp::sensor::fradia_sensor <lib::empty_t, chessboard_t, eihcalibration_t> *fradia;
	boost::shared_ptr <mrrocpp::ecp_mp::sensor::discode::discode_sensor> sensor;
protected:
	std::string K_fp;
	std::string kk_fp;
	std::string M_fp;
	std::string mm_fp;

	// generator do wodzenia za nos
	generator::eih_nose_run* nose;
	// generator smooth
	generator::newsmooth* smoothgen;
	// generator do wysylania danych do fradii
	generator::eihgenerator* generator;
	bool store_data(void);
	void main_task_algorithm(void);

	Types::Mrrocpp_Proxy::EIHReading reading;

	virtual void retrieve_reading();
	virtual bool is_object_visible_in_latest_reading();

public:
	virtual Types::Mrrocpp_Proxy::EIHReading* get_reading();

	void conditional_execution();
	acq_eih(task::task &_ecp_t, boost::shared_ptr <mrrocpp::ecp_mp::sensor::discode::discode_sensor> sensor);
	void write_data(const std::string & _K_fp, const std::string & _k_fp, const std::string & _M_fp, const std::string & _m_fp, int _number_of_measures);
};

}
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif
