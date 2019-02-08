/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"

#include <iostream>

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/commands/Scheduler.h>

std::unique_ptr< DriveTrain > Robot::m_driveTrain{};
std::unique_ptr< Input > Robot::m_input{};

std::unique_ptr< ManualControl > Robot::m_manualControl{};
std::unique_ptr< ApproachCargo > Robot::m_approachCargo{};
std::unique_ptr< SpeedTest > Robot::m_speedTest{};
std::unique_ptr< FollowPath > Robot::m_followPath{};

std::unique_ptr< Arm > Robot::m_arm{};

std::unique_ptr< ManualArm > Robot::m_manualArm{};

void Robot::RobotInit() {
  m_driveTrain = std::make_unique< DriveTrain >(3, 5, 7, 4, 6, 8);
  m_input = std::make_unique< Input >(1, 2);

	m_manualControl = std::make_unique< ManualControl >(Robot::m_input.get());
	m_approachCargo = std::make_unique< ApproachCargo >(10);
	m_speedTest = std::make_unique< SpeedTest >(Robot::m_input.get());

  m_arm = std::make_unique< Arm >(1, 0, 2, 1);

  m_manualArm = std::make_unique< ManualArm >(Robot::m_input.get());
}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {
  auto armSensors = m_arm->getSensorValues();

  frc::SmartDashboard::PutNumber("Arm Base Analog", armSensors.first);
  frc::SmartDashboard::PutNumber("Arm Wrist Analog", armSensors.second);
}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
void Robot::AutonomousInit() {}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {
  m_manualArm->Start();
}

void Robot::TeleopPeriodic() {
  std::cout << m_arm->getLevel() << "\n";
  frc::Scheduler::GetInstance()->Run();
}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
