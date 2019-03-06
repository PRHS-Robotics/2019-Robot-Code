/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/commands/Command.h>

class DriveDistance : public frc::Command {
public:
  DriveDistance(double dist);
  void Initialize() override;
  void Execute() override;
  bool IsFinished() override;
  void End() override;
  void Interrupted() override;

  double targetDistance = 0.0;

  double distanceError();

private:
  double m_encoderOffset = 0.0;
  double m_angle = 0.0;
  
  double m_speed = 0.0;

  double averageEncoderValue();
  double targetSpeed();
};
