/*
 * Clamp.h
 *
 *  Created on: Nov 27, 2018
 *      Author: Ben
 */

#ifndef SRC_SUBSYSTEMS_CLAMP_H_
#define SRC_SUBSYSTEMS_CLAMP_H_

#include <ctre/Phoenix.h>
#include <SpeedControllerGroup.h>
#include <Drive/DifferentialDrive.h>
class Clamp {
public:

	Clamp();
	virtual ~Clamp();
};
private:
	WPI_TalonSRX screwD;
#endif /* SRC_SUBSYSTEMS_CLAMP_H_*/
