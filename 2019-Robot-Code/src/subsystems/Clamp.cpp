/*
 * DriveTrain.cpp
 *
 *  Created on: Nov 29, 2018
 *      Author: Ben
 */

#include "subsystems/Clamp.h"
#include <ctre/Phoenix.h>
#include <SpeedControllerGroup.h>
#include <Drive/DifferentialDrive.h>
#include <SmartDashboard/SmartDashboard.h>


class Clamp {
public:

	Clamp(unsigned int motorId);

	void screw(double speed);

	void setInverted(bool inverted);

	WPI_TalonSRX m_screwMotor;

	static double m_sensitivity;
	static bool m_inverted = true;
	static double m_downMultiplier = 0.5;
};

Clamp::Clamp(unsigned int motorId) :
	m_screwMotor(motorId)
{
	setInverted(m_inverted);
}

void Clamp::screw(double speed) {
	frc::SmartDashboard::PutNumber("Screw Sensitivity", m_sensitivity);

	if (speed < 0){
		speed *= m_downMultiplier = 0.5;
	}

	frc::SmartDashboard::PutNumber("screwPower", speed);

	m_screwMotor.Set(speed);
}


void Clamp::setInverted(bool inverted) {
	m_inverted = inverted;
	m_screwMotor.SetInverted(inverted);
}
