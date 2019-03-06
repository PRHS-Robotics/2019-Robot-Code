/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include "subsystems/DriveTrain.h"
#include "subsystems/ElevatorDriveTrain.h"
#include "subsystems/Input.h"
#include "subsystems/Arm.h"
#include "subsystems/Manipulator.h"

#include <string>
#include <memory>

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/Compressor.h>
#include <frc/SerialPort.h>
#include <frc/DigitalOutput.h>
#include <frc/AnalogInput.h>

#include "commands/auto/ApproachCargo.h"
#include "commands/auto/FollowPath.h"
#include "commands/auto/DriveUntil.h"
#include "commands/auto/LevelDriveUntil.h"
#include "commands/auto/TurnToAngle.h"
#include "commands/ManualControl.h"
#include "commands/SpeedTest.h"
#include "commands/ManualArm.h"
#include "commands/ManualManip.h"
#include "commands/Elevator.h"
#include "commands/CalibrateArm.h"

class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;

  static double getYaw();
  static double getHeading();

  void matchInit();
  void matchPeriodic();

  frc::SendableChooser< int > m_testModeChooser;

  frc::SendableChooser< int > m_armChooser;
  frc::SendableChooser< int > m_manipulatorChooser;
  frc::SendableChooser< int > m_driveTrainChooser;
  frc::SendableChooser< int > m_pneumaticChooser;

  static std::unique_ptr< frc::Compressor > m_compressor;

  static std::unique_ptr< DriveTrain > m_driveTrain;
  static std::unique_ptr< ElevatorDriveTrain > m_elevatordrivetrain;
  static std::unique_ptr< Input > m_input;

	static std::unique_ptr< ManualControl > m_manualControl;
	static std::unique_ptr< ApproachCargo > m_approachCargo;
	static std::unique_ptr< SpeedTest > m_speedTest;
	static std::unique_ptr< FollowPath > m_followPath;

  static std::unique_ptr< Arm > m_arm;
  static std::unique_ptr< Manipulator > m_manipulator;
  static std::unique_ptr< Elevator > m_elevator;

  static std::unique_ptr< ManualArm > m_manualArm;
  static std::unique_ptr< ManualManip > m_manualManip;

  static std::unique_ptr< PigeonIMU > m_gyro;

  static std::unique_ptr< CalibrateArm > m_calibrateArm;

  static std::unique_ptr< frc::SerialPort > m_cameraSerial;

  static std::unique_ptr< frc::DigitalOutput > m_lights;

  static std::unique_ptr< frc::AnalogInput > m_ultrasonic;

  static std::unique_ptr< DriveUntil > m_driveUntil;

  static std::unique_ptr< LevelDriveUntil > m_levelDriveUntil;

  static std::unique_ptr< TurnToAngle > m_turnToAngle;
};

double constrainAngle(double angle);

double minDifference(double startAngle, double endAngle);