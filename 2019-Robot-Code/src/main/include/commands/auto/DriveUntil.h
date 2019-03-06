/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/commands/Command.h>
#include <frc/PIDController.h>

class DriveUntil : public frc::Command {
public:
  DriveUntil(double targetDistance);
  void Initialize() override;
  void Execute() override;
  bool IsFinished() override;
  void End() override;
  void Interrupted() override;

  double m_targetDistance;
  
private:
  /*class DriveTrainOutput : public frc::PIDOutput {
  public:
    void PIDWrite(double output) override;
  };

  class UltrasonicInput : public frc::PIDSource {
  public:
    UltrasonicInput();

    double PIDGet() override;

    ~UltrasonicInput() override = default;

  private:
    double m_lastDistance;
  };

  DriveTrainOutput m_pidOutput;
  UltrasonicInput m_pidInput;

  frc::PIDController m_controller;*/

};
