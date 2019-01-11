/*
 * Configuration.cpp
 *
 *  Created on: Jan 3, 2019
 *      Author: super-tails
 */

#include "Configuration.h"
#include <SmartDashboard/SmartDashboard.h>

template < typename T >
Option<T>::Option(std::string displayName, const T& defaultValue, validator_type validator) :
	m_displayName(displayName),
	m_defaultValue(defaultValue)/*,
	m_validator(validator)*/
{

}

template < typename T >
T Option<T>::get() const {
	return T{};
}
