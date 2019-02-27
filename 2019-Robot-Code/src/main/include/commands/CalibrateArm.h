/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/commands/Command.h>
#include "subsystems/Input.h"

class CalibrateArm : public frc::Command {
public:
  CalibrateArm(Input *input);
  void Initialize() override;
  void Execute() override;
  bool IsFinished() override;
  void End() override;
  void Interrupted() override;

private:
  Input *m_input;

  static constexpr const std::size_t LEVEL_COUNT = 10;

  std::array< double, LEVEL_COUNT > m_baseSensorValues = { 0 };
  std::array< double, LEVEL_COUNT > m_wristSensorValues = { 0 };

  void saveValues();
  void readValues();

  bool m_debounce = false;
};
