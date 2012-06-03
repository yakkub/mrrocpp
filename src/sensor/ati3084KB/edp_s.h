/*!
 * @file
 * @brief File containing the declaration of the ATI3084 Froce/Torque sensor class.
 *
 * @author Konrad Banachowicz
 *
 */

#if !defined(_EDP_S_ATI3084_KB_H)
#define _EDP_S_ATI3084_KB_H

#include <comedilib.h>
#include <Eigen/Core>

// import most common Eigen types
USING_PART_OF_NAMESPACE_EIGEN

#include "base/edp/edp_force_sensor.h"

namespace mrrocpp {
namespace edp {
namespace sensor {

typedef Matrix <double, 6, 6> Matrix6d;
typedef Matrix <double, 6, 1> Vector6d;


/*!
 *
 * @brief ATI3084 Force/Torque sensor class.
 *
 * @author Konrad Banachowicz
 *
 */
class ATI3084_force : public force
{
public:

	void connect_to_hardware(void);

	ATI3084_force(common::manip_effector &_master);
	virtual ~ATI3084_force();
	void disconnect_from_hardware(void);
	void configure_particular_sensor(void);
	void wait_for_particular_event(void);
	void get_particular_reading(void);

private:
	const std::string dev_name;
	comedi_t *device; // device descriptor
	lsampl_t adc_data[6]; // raw ADC data
	Vector6d datav; // mensured voltage
	Vector6d bias_data; // sensor bias voltage

	lsampl_t maxdata;
	comedi_range *rangetype;

	comedi_polynomial_t ADC_calib[6]; // ADC calibration polynomial

	Matrix6d conversion_matrix; // F/T conversion matrix
	Vector6d conversion_scale; // F/T scaling

	void convert_data(const Vector6d &result_raw, const Vector6d &bias_raw, lib::Ft_vector &force) const;
};

} // namespace sensor
} // namespace edp
} // namespace mrrocpp


#endif //_EDP_S_ATI3084_KB_H

