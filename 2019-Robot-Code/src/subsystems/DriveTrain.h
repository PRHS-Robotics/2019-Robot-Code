/*
 * DriveTrain.h
 *
 *  Created on: Nov 10, 2018
 *      Author: super-tails
 */

#ifndef SRC_SUBSYSTEMS_DRIVETRAIN_H_
#define SRC_SUBSYSTEMS_DRIVETRAIN_H_

#include "subsystems/Input.h"

#include <memory>

#include <ctre/Phoenix.h>
#include <SpeedControllerGroup.h>
#include <Drive/DifferentialDrive.h>

class DriveTrain {
public:

	DriveTrain(int frontLeft, int backLeft, int frontRight, int backRight);

	void drive(InputState state);

private:

	WPI_TalonSRX m_frontLeft, m_backLeft, m_frontRight, m_backRight;

	frc::DifferentialDrive m_drive;

};



#endif /* SRC_SUBSYSTEMS_DRIVETRAIN_H_ */
