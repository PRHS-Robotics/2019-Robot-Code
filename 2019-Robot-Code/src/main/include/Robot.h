/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include "subsystems/DriveTrain.h"
#include "subsystems/Input.h"
#include "subsystems/Arm.h"

#include <string>
#include <memory>

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>

#include "commands/ApproachCargo.h"
#include "commands/FollowPath.h"
#include "commands/ManualControl.h"
#include "commands/SpeedTest.h"
#include "commands/ManualArm.h"
#include "commands/ManualManip.h"

class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;
  
  std::string m_autoSelected;

  static std::unique_ptr< DriveTrain > m_driveTrain;
  static std::unique_ptr< Input > m_input;

	static std::unique_ptr< ManualControl > m_manualControl;
	static std::unique_ptr< ApproachCargo > m_approachCargo;
	static std::unique_ptr< SpeedTest > m_speedTest;
	static std::unique_ptr< FollowPath > m_followPath;

  static std::unique_ptr< Arm > m_arm;
  static std::unique_ptr< Manipulator > m_manipulator;

  static std::unique_ptr< ManualArm > m_manualArm;
  static std::unique_ptr< ManualManip > m_manualManip;
};
