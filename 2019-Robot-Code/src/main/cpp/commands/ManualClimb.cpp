/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/ManualClimb.h"
#include "Robot.h"

ManualClimb::ManualClimb(Input *input) :
  m_input(input),
  frc::Command("Climb", *Robot::m_elevatorDriveTrain) {

}

// Called just before this Command runs the first time
void ManualClimb::Initialize() {

}

// Called repeatedly when this Command is scheduled to run
void ManualClimb::Execute() {
  Robot::m_elevatorDriveTrain->extend(m_input->getInput().ry);
  Robot::m_elevatorDriveTrain->drive(m_input->getInput().y);
}

// Make this return true when this Command no longer needs to run execute()
bool ManualClimb::IsFinished() {
  return false;
}

// Called once after isFinished returns true
void ManualClimb::End() {
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void ManualClimb::Interrupted() {

}
