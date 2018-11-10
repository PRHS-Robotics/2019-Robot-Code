/*
 * DriveTrain.h
 *
 *  Created on: Nov 10, 2018
 *      Author: super-tails
 */

#ifndef SRC_SUBSYSTEMS_DRIVETRAIN_H_
#define SRC_SUBSYSTEMS_DRIVETRAIN_H_

#include "subsystems/Input.h"

#include <Talon.h>
#include <SpeedControllerGroup.h>
#include <Drive/DifferentialDrive.h>

class DriveTrain {
public:

	DriveTrain(int frontLeft, int backLeft, int frontRight, int backRight);

	void drive(InputState state);

private:

	frc::Talon m_frontLeft;
	frc::Talon m_frontRight;
	frc::Talon m_backLeft;
	frc::Talon m_backRight;

	frc::SpeedControllerGroup m_lMotors;
	frc::SpeedControllerGroup m_rMotors;

	frc::DifferentialDrive m_drive;
};



#endif /* SRC_SUBSYSTEMS_DRIVETRAIN_H_ */
