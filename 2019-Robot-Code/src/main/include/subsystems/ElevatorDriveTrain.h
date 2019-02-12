/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
#ifndef SRC_SUBSYSTEMS_ELEVATORDRIVETRAIN_H_
#define SRC_SUBSYSTEMS_ELEVATORDRIVETRAIN_H_

#include "subsystems/Input.h"
#include "rev/SparkMax.h"
#include <frc/commands/Subsystem.h>

class ElevatorDriveTrain : public frc::Subsystem {
public:
  ElevatorDriveTrain(int extender, int driveMotor);

  void drive();

private:
  rev::SparkMax m_extender;
  rev::SparkMax m_driveMotor;
};
#endif /*SRC_SUBSYSTEMS_ELEVATORDRIVETRAIN_H_*/
