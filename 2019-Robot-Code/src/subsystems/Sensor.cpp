/*
 * Sensor.cpp
 *
 *  Created on: Nov 17, 2018
 *      Author: super-tails
 */

#include "Sensor.h"

SensorNetwork::SensorNetwork(int address) :
	m_i2c(frc::I2C::Port::kOnboard) {
}

DataFrame SensorNetwork::readAll() {
	return DataFrame{};
}
