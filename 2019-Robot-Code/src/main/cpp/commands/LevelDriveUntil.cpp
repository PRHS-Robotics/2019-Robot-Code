/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/LevelDriveUntil.h"
#include "commands/ManualArm.h"
#include "Robot.h"

double levelToDistance(Level level) {
  switch (level) {
  case Level::CargoLevel1:
    return /*40.0*/50.0;
  case Level::CargoLevel2:
    return 40.0;
  case Level::CargoLevel3:
    return 23.0;
  case Level::HatchLevel1:
    return 20.0;
  case Level::HatchLevel2:
    return 34.0;
  case Level::HatchLevel3:
    return 22.0;
  default:
    return 40.0;
  }
}

LevelDriveUntil::LevelDriveUntil() :
  DriveUntil(levelToDistance(Robot::m_arm->getLevel()))
{

}

void LevelDriveUntil::Initialize() {
  m_targetDistance = levelToDistance(Robot::m_arm->getLevel());
}