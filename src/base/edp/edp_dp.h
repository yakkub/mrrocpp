/*!
 * \file edp_dp.h
 * \brief File containing the declaration of edp data port class
 *
 * \author yoyek
 *
 */

#ifndef __EDP_DP_H
#define __EDP_DP_H

#include <boost/utility.hpp>



namespace mrrocpp {
namespace edp {
namespace common {

template <class T>
class edp_dp
{
protected:
	/**
	 * @brief no data flag
	 * it is set if no data is stored (before first set method call)
	 */
	bool no_data;

//	T _data;

public:
	/**
	 * @brief data stored basing on template
	 */
	T data;

	/*
	 T & data() {
	 set();
	 return _data;
	 }

	 const T & data() const
	 {
	 return _data;
	 }
	 */

	/**
	 * @brief Constructor
	 * @param _name Unique port name.
	 * @param _port_manager port manager reference.
	 */
	edp_dp() :
	no_data(true)
	{
	}


};

} // namespace common
} // namespace edp
} // namespace mrrocpp

#endif
