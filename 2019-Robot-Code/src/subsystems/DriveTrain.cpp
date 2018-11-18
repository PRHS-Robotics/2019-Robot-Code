/*
 * DriveTrain.cpp
 *
 *  Created on: Nov 10, 2018
 *      Author: super-tails
 */

#include "subsystems/DriveTrain.h"
#include <iostream>

DriveTrain::DriveTrain(int frontLeft, int midLeft, int backLeft, int frontRight, int midRight, int backRight) :
	m_frontLeft(frontLeft),
	m_midLeft(midLeft),
	m_backLeft(backLeft),
	m_frontRight(frontRight),
	m_midRight(midRight),
	m_backRight(backRight),
	m_drive(m_frontLeft, m_frontRight)
{
	m_midLeft.Follow(m_frontLeft);
	m_backLeft.Follow(m_frontLeft);

	m_midRight.Follow(m_frontRight);
	m_backRight.Follow(m_frontRight);
}

void DriveTrain::drive(InputState state) {
	double lSpeed = -state.y + state.r;
	double rSpeed = -state.y - state.r;
	m_drive.TankDrive(-lSpeed, -rSpeed);
}
