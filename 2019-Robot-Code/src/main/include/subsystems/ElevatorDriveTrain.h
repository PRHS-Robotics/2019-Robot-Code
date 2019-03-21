/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
#ifndef SRC_SUBSYSTEMS_ELEVATORDRIVETRAIN_H_
#define SRC_SUBSYSTEMS_ELEVATORDRIVETRAIN_H_

#include "subsystems/Input.h"
#include <frc/AnalogInput.h>
#include "rev/SparkMax.h"
#include "subsystems/ElevatorDriveTrain.h"
#include <frc/commands/Subsystem.h>

class ElevatorDriveTrain : public frc::Subsystem {
public:
  ElevatorDriveTrain(int extender, int driveMotor, int extendSolenoid, int retractSolenoid, int heightSensor);

  void drive(double speed);

  void extend(double speed);

  void setSolenoid(bool state);

  double getHeight();

private:
  rev::SparkMax m_extender;
  rev::SparkMax m_driveMotor;

  frc::Solenoid m_extendSolenoid;
  frc::Solenoid m_retractSolenoid;

  frc::AnalogInput m_heightSensor;
};
#endif /*SRC_SUBSYSTEMS_ELEVATORDRIVETRAIN_H_*/
