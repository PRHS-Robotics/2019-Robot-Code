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

	DriveTrain(int frontLeft, int midLeft, int backLeft, int frontRight, int midRight, int backRight);

	void drive(InputState state);

private:

	WPI_TalonSRX m_frontLeft;
	WPI_TalonSRX m_midLeft;
	WPI_TalonSRX m_backLeft;

	WPI_TalonSRX m_frontRight;
	WPI_TalonSRX m_midRight;
	WPI_TalonSRX m_backRight;

	frc::DifferentialDrive m_drive;
};



#endif /* SRC_SUBSYSTEMS_DRIVETRAIN_H_ */
