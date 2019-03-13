/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/auto/SetLevel.h"

#include "Robot.h"

SetLevel::SetLevel(Level level) :
  m_level(level),
  frc::Command(*Robot::m_arm)
{
  // Use Requires() here to declare subsystem dependencies
  // eg. Requires(Robot::chassis.get());
}

// Called just before this Command runs the first time
void SetLevel::Initialize() {
  Robot::m_arm->setLevel(m_level);
}

// Called repeatedly when this Command is scheduled to run
void SetLevel::Execute() {}

// Make this return true when this Command no longer needs to run execute()
bool SetLevel::IsFinished() {
  return Robot::m_arm->setpointReached();
}

// Called once after isFinished returns true
void SetLevel::End() {}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void SetLevel::Interrupted() {}
