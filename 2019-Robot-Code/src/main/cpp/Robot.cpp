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
#include <cameraserver/CameraServer.h>

std::unique_ptr< DriveTrain > Robot::m_driveTrain{};
std::unique_ptr< Input > Robot::m_input{};
std::unique_ptr< frc::Compressor > Robot::m_compressor{};

std::unique_ptr< ManualControl > Robot::m_manualControl{};
std::unique_ptr< ApproachCargo > Robot::m_approachCargo{};
std::unique_ptr< SpeedTest > Robot::m_speedTest{};
std::unique_ptr< FollowPath > Robot::m_followPath{};
std::unique_ptr< Elevator > Robot::m_elevator{};
std::unique_ptr< ElevatorDriveTrain > Robot::m_elevatordrivetrain{};
std::unique_ptr< Manipulator > Robot::m_manipulator{};
std::unique_ptr< Arm > Robot::m_arm{};

std::unique_ptr< ManualManip > Robot::m_manualManip{};

std::unique_ptr< ManualArm > Robot::m_manualArm{};


void Robot::RobotInit() {
  m_driveTrain = std::make_unique< DriveTrain >(3, 5, 7, 4, 6, 8);
  m_input = std::make_unique< Input >(0, 1);

	m_manualControl = std::make_unique< ManualControl >(Robot::m_input.get());
	m_approachCargo = std::make_unique< ApproachCargo >(10);
	m_speedTest = std::make_unique< SpeedTest >(Robot::m_input.get());

  m_arm = std::make_unique< Arm >(1, 0, 2, 1, 1);

  m_manualArm = std::make_unique< ManualArm >(Robot::m_input.get());

  m_manipulator = std::make_unique< Manipulator >(3, 0, 1, 0);

  m_manualManip = std::make_unique< ManualManip >(Robot::m_input.get());

  m_compressor = std::make_unique< frc::Compressor >(0);

  m_testModeChooser.SetDefaultOption("Competition Mode", 0);
  m_testModeChooser.AddOption("Test Mode", 1);

  m_armChooser.SetDefaultOption("DisabledArm", 0);
  m_armChooser.AddOption("EnabledArm", 1);

  m_manipulatorChooser.SetDefaultOption("DisabledManip", 0);
  m_manipulatorChooser.AddOption("EnabledManip", 1);

  m_driveTrainChooser.SetDefaultOption("DisabledDrive", 0);
  m_driveTrainChooser.AddOption("EnabledDrive", 1);

  m_pneumaticChooser.SetDefaultOption("DisabledCompressor", 0);
  m_pneumaticChooser.AddOption("EnabledCompressor", 1);

  frc::SmartDashboard::PutData("Mode Chooser", &m_testModeChooser);
  frc::SmartDashboard::PutData("Arm Chooser", &m_armChooser);
  frc::SmartDashboard::PutData("Manip Chooser", &m_manipulatorChooser);
  frc::SmartDashboard::PutData("Drive Chooser", &m_driveTrainChooser);
  frc::SmartDashboard::PutData("Pneumatic Chooser", &m_pneumaticChooser);

  frc::CameraServer::GetInstance()->StartAutomaticCapture(0);
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

void Robot::matchInit() {
  frc::Scheduler::GetInstance()->RemoveAll();

  std::cout << "Starting teleop\n";
  if (m_testModeChooser.GetSelected()) {
    std::cout << "Running in test mode\n";
    if (m_armChooser.GetSelected()) {
      std::cout << "Starting arm control system\n";
      m_manualArm->Start();
    }
    if (m_manipulatorChooser.GetSelected()) {
      std::cout << "Starting manipulator control system\n";
      m_manualManip->Start();
    }
    if (m_driveTrainChooser.GetSelected()) {
      std::cout << "Starting drive train control system\n";
      m_manualControl->Start();
    }
    if (m_pneumaticChooser.GetSelected()) {
      std::cout << "Starting pneumatic system\n";
      m_compressor->Start();
    }
    else {
      m_compressor->Stop();
    }
    std::cout << "Finished starting commands\n";
  }
  else {
    std::cout << "Running in competition mode\n";
    m_manualControl->Start();
    m_manualArm->Start();
    m_manualManip->Start();
    m_compressor->Start();
  }
}

void Robot::matchPeriodic() {
  frc::Scheduler::GetInstance()->Run();
}

void Robot::AutonomousInit() {
  matchInit();
}

void Robot::AutonomousPeriodic() {
  matchPeriodic();
}

void Robot::TeleopInit() {
  matchInit();
}

void Robot::TeleopPeriodic() {
  matchPeriodic();
}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
