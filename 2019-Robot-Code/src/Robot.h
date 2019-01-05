/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <memory>

#include <string>

#include <IterativeRobot.h>
#include <SmartDashboard/SendableChooser.h>
#include <SerialPort.h>
#include <AnalogInput.h>
#include <Compressor.h>
#include <SmartDashboard/SmartDashboard.h>

class DriveTrain;
class Input;
class Autonomous;

class Robot : public frc::IterativeRobot {
public:
	void RobotInit() override;
	void AutonomousInit() override;
	void AutonomousPeriodic() override;
	void TeleopInit() override;
	void TeleopPeriodic() override;
	void TestPeriodic() override;

private:
	frc::SendableChooser<std::string> m_chooser;
	const std::string kAutoNameDefault = "Default";
	const std::string kAutoNameCustom = "My Auto";
	std::string m_autoSelected;

	std::unique_ptr< DriveTrain > m_driveTrain;
	std::unique_ptr< Input > m_input;
	std::unique_ptr< Autonomous > m_autonomous;
	std::unique_ptr< frc::SerialPort > m_serialPort;
	std::unique_ptr< frc::AnalogInput > m_analogInput;
	std::unique_ptr< frc::Compressor > m_compressor;
};
