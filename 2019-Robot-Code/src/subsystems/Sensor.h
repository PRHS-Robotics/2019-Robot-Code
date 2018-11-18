/*
 * Sensor.h
 *
 *  Created on: Nov 17, 2018
 *      Author: super-tails
 */

#ifndef SRC_SUBSYSTEMS_SENSOR_H_
#define SRC_SUBSYSTEMS_SENSOR_H_

#include <I2C.h>
#include <stdint.h>

struct DataFrame {
};

class SensorNetwork {
public:
	SensorNetwork(int address);

	DataFrame readAll();

private:

	frc::I2C m_i2c;
};

#endif /* SRC_SUBSYSTEMS_SENSOR_H_ */
