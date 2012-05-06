/*
 * EIHReading.hpp
 *
 *  Created on: 06-05-2012
 *      Author: jkosiore
 */

#ifndef EIHReading_HPP_
#define EIHReading_HPP_

#include <sstream>

#include "Reading.h"
#include "HomogMatrix.h"

namespace Types {
namespace Mrrocpp_Proxy {

/**
 *
 */
class EIHReading: public Reading
{
public:
	EIHReading() : objectVisible(false)
	{
	}

	virtual ~EIHReading()
	{
	}

	virtual EIHReading* clone()
	{
		return new EIHReading(*this);
	}

	bool objectVisible;
	Types::HomogMatrix objectPosition;

	virtual void send(boost::shared_ptr<xdr_oarchive<> > & ar){
		*ar<<*this;
	}

private:
	friend class boost::serialization::access;
	template <class Archive>
	void serialize(Archive & ar, const unsigned int version)
	{
//		std::cout << "EIHReading::serialize()\n";
		ar & boost::serialization::base_object <Reading>(*this);

		ar & objectVisible;
		ar & objectPosition;
	}
};

}//namespace Mrrocpp_Proxy
}//namespace Types

#endif /* EIHReading_HPP_ */
