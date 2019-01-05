/*
 * Configuration.cpp
 *
 *  Created on: Jan 3, 2019
 *      Author: super-tails
 */

#include "Configuration.h"
#include <SmartDashboard/SmartDashboard.h>

template < typename T >
Option<T>::Option(std::string displayName, const T& defaultValue) :
	m_displayName(displayName),
	m_defaultValue(defaultValue)
{

}

template < typename T >
T Option<T>::get() const {
	return T{};
}
