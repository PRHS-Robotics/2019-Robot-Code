/*
 * DriveTrain.cpp
 *
 *  Created on: Nov 10, 2018
 *      Author: super-tails
 */

#include "subsystems/DriveTrain.h"
#include "Robot.h"
#include <iostream>
#include <algorithm>
#include <SmartDashboard/SmartDashboard.h>
#include <frc/commands/Scheduler.h>

bool DriveTrain::leftSidePhase() const {
	return frc::SmartDashboard::GetBoolean("Left Sensor Phase", false);
}

void DriveTrain::setLeftSidePhase(bool phase) {
	frc::SmartDashboard::SetPersistent("Left Sensor Phase");
	frc::SmartDashboard::PutBoolean("Left Sensor Phase", phase);

	for (auto& talon : m_lMotors) {
		talon->SetSensorPhase(phase);
	}
}

bool DriveTrain::rightSidePhase() const {
	return frc::SmartDashboard::GetBoolean("Right Sensor Phase", false);
}

void DriveTrain::setRightSidePhase(bool phase) {
	frc::SmartDashboard::SetPersistent("Right Sensor Phase");
	frc::SmartDashboard::PutBoolean("Right Sensor Phase", phase);

	for (auto& talon : m_rMotors) {
		talon->SetSensorPhase(phase);
	}
}

std::pair< std::array< int, 3 >, std::array< int, 3 > > DriveTrain::getEncoderPositions() {
	std::array< int, 3 > lPositions;
	std::array< int, 3 > rPositions;
	for (int i = 0; i < 3; ++i) {
		lPositions[i] = m_lMotors[i]->GetSelectedSensorPosition();
		rPositions[i] = m_rMotors[i]->GetSelectedSensorPosition();
	}
	return { lPositions, rPositions };
}

#define M(X) std::make_unique< WPI_TalonSRX >(X)

DriveTrain::DriveTrain(int frontLeft, int midLeft, int backLeft, int frontRight, int midRight, int backRight) :
	m_lMotors{ M(frontLeft), M(midLeft), M(backLeft) },
	m_rMotors{ M(frontRight), M(midRight), M(backRight) },
	Subsystem("DriveTrain")
{
	if (!frc::SmartDashboard::SetDefaultBoolean("Left Sensor Phase", false)) {
		std::cout << "Setting default left side phase\n";
		setLeftSidePhase(false);
	}
	else {
		setLeftSidePhase(leftSidePhase());
	}
	if (!frc::SmartDashboard::SetDefaultBoolean("Right Sensor Phase", false)) {
		std::cout << "Setting default right side phase\n";
		setRightSidePhase(false);
	}
	else {
		setRightSidePhase(rightSidePhase());
		std::cout << "Setting default right side phase\n";
		setRightSidePhase(false);
	}

	for (auto& motor : m_lMotors) {
		motor->SetInverted(true);
	}

	/*
	for (int i = 0; i < 3; ++i) {
		m_lMotors[i].SetInverted(true);
	}
	*/

	// Configure both front left and front right talons identically

	for (auto& talon : m_lMotors) {
		talon->ConfigNominalOutputForward(0, 10);
		talon->ConfigNominalOutputReverse(0, 10);
		talon->ConfigPeakOutputForward(1, 10);
		talon->ConfigPeakOutputReverse(-1, 10);

		talon->ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0, 10);
		talon->ConfigClosedloopRamp(0.0, 10);
	}
	for (auto& talon : m_rMotors) {
		talon->ConfigNominalOutputForward(0, 10);
		talon->ConfigNominalOutputReverse(0, 10);
		talon->ConfigPeakOutputForward(1, 10);
		talon->ConfigPeakOutputReverse(-1, 10);

		talon->ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0, 10);
		talon->ConfigClosedloopRamp(0.0, 10);
	}

	/*
	m_frontLeft.Config_kF(0, 16.771, 10);
	m_frontLeft.Config_kP(0, 16.771 / 2.0, 10);
	m_frontLeft.Config_kI(0, 0, 10);
	m_frontLeft.Config_kD(0, 0, 10);

	m_frontRight.Config_kF(0, 16.119, 10);
	m_frontRight.Config_kP(0, 16.119 / 2.0, 10);
	m_frontRight.Config_kI(0, 0 , 10);
	m_frontRight.Config_kD(0, 0, 10);
	
	m_frontLeft.SetSensorPhase(true);
	m_frontRight.SetSensorPhase(true);
	*/

	frc::Scheduler::GetInstance()->RegisterSubsystem(this);
}

/*void DriveTrain::InitDefaultCommand() {
	SetDefaultCommand(Robot::m_manualControl.get());
}*/

void DriveTrain::resetSensors() {
	for (auto& talon : m_lMotors) {
		talon->SetSelectedSensorPosition(0, 0, 10);
	}
	for (auto& talon : m_rMotors) {
		talon->SetSelectedSensorPosition(0, 0, 10);
	}
}

void DriveTrain::drive(InputState state) {
	double lSpeed = -state.y + state.r;
	double rSpeed = -state.y - state.r;
	lSpeed = std::max(std::min(lSpeed, 1.0), -1.0);
	rSpeed = std::max(std::min(rSpeed, 1.0), -1.0);

	drive(lSpeed, rSpeed, !buttonValue(state, "TRIGGER"));
}

template < typename T >
int signum(const T& value) {
	return (value > 0) - (value < 0);
}

/*void DriveTrain::calibratePhase(double leftSpeed, double rightSpeed) {
	// Allow time for measured velocity to change after inverting a phase
	static int debounce = 0;

	if (debounce > 0) {
		--debounce;
		return;
	}

	double leftVelocity = m_frontLeft.GetSelectedSensorVelocity();
	if (std::abs(leftSpeed) > 0.5 && std::abs(leftVelocity) > 10 && signum(leftVelocity) != signum(leftSpeed)) {
		bool currentPhase = leftSidePhase();
		std::cout << "Left side sensor phase '" << std::boolalpha << currentPhase << "' incorrect, inverting...\n";
		setLeftSidePhase(!currentPhase);
		debounce = 1000;
	}

	double rightVelocity = m_frontRight.GetSelectedSensorVelocity();
	if (std::abs(rightSpeed) > 0.5 && std::abs(rightVelocity) > 10 && signum(rightVelocity) != signum(rightSpeed)) {
		bool currentPhase = rightSidePhase();
		std::cout << "Right side sensor phase '" << std::boolalpha << currentPhase << "' incorrect, inverting...\n";
		setRightSidePhase(!currentPhase);
		debounce = 1000;
	}
}*/

void DriveTrain::drive(double leftSpeed, double rightSpeed, bool percentOutput) {
	//calibratePhase(leftSpeed, rightSpeed);

	const double MAX_SPEED = 50.0;
	
	const double MAX_PERCENT = 1.0;

	if (percentOutput) {
		for (auto& motor : m_lMotors) {
			motor->Set(ControlMode::PercentOutput, leftSpeed * MAX_PERCENT);
		}
		for (auto& motor : m_rMotors) {
			motor->Set(ControlMode::PercentOutput, rightSpeed * MAX_PERCENT);
		}
	}
	else {
		for (auto& motor : m_lMotors) {
			motor->Set(ControlMode::Velocity, leftSpeed * MAX_SPEED);
		}
		for (auto& motor : m_rMotors) {
			motor->Set(ControlMode::Velocity, rightSpeed * MAX_SPEED);
		}
	}

	static std::array< MovingAverage, 3 > lVelocity{ MovingAverage{30}, {30}, {30} };
	static std::array< MovingAverage, 3 > rVelocity{ MovingAverage{30}, {30}, {30} };
	for (int i = 0; i < 3; ++i) {
		std::string lKey = "Left Side Velocity " + i;
		std::string rKey = "Right Side Velocity " + i;
		double lValue = m_lMotors[i]->GetSelectedSensorVelocity(0);
		double rValue = m_rMotors[i]->GetSelectedSensorVelocity(0);
		frc::SmartDashboard::PutNumber(lKey, lVelocity[i].Process(lValue));
		frc::SmartDashboard::PutNumber(rKey, rVelocity[i].Process(rValue));
	}

	frc::SmartDashboard::PutNumber("Left Side Set Speed", leftSpeed);
	frc::SmartDashboard::PutNumber("Right Side Set Speed", rightSpeed);

	//frc::SmartDashboard::PutNumber("Left Side Error", m_frontLeft.GetClosedLoopError(0));
	//frc::SmartDashboard::PutNumber("Right Side Error", m_frontLeft.GetClosedLoopError(0));
}
