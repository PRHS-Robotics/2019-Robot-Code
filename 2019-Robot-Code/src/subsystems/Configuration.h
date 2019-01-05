/*
 * Configuration.h
 *
 *  Created on: Jan 3, 2019
 *      Author: super-tails
 */

#ifndef SRC_SUBSYSTEMS_CONFIGURATION_H_
#define SRC_SUBSYSTEMS_CONFIGURATION_H_

#include <string>

template < typename T >
struct Option {
	std::string m_displayName;
	const T& m_defaultValue;

	Option() = delete;
	Option(const Option&) = delete;
	Option(Option&&) = default;

	Option(std::string displayName, const T& defaultValue);

	T get() const;
};



#endif /* SRC_SUBSYSTEMS_CONFIGURATION_H_ */
