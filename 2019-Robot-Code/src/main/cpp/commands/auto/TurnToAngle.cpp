/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/auto/TurnToAngle.h"
#include "Robot.h"
#include <iostream>
#include <algorithm>

TurnToAngle::TurnToAngle(double target) :
  m_target(constrainAngle(target)),
  frc::Command("TurnToAngle", *Robot::m_driveTrain) {
  // Use Requires() here to declare subsystem dependencies
  // eg. Requires(Robot::chassis.get());
}

void TurnToAngle::setTarget(double angle) {
  m_target = constrainAngle(angle);
}

double TurnToAngle::getTarget() const {
  return m_target;
}

// Called just before this Command runs the first time
void TurnToAngle::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void TurnToAngle::Execute() {
  double angle_difference = minDifference(Robot::getHeading(), m_target);
  
  double turn = 1.0 * (-1.0/80.0) * angle_difference;

  Robot::m_driveTrain->drive(turn, -turn);
}

// Make this return true when this Command no longer needs to run execute()
bool TurnToAngle::IsFinished() {
  return std::abs(minDifference(Robot::getHeading(), m_target)) <= 0.1;
}

// Called once after isFinished returns true
void TurnToAngle::End() {
  Robot::m_driveTrain->drive(0.0, 0.0);
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void TurnToAngle::Interrupted() {
  End();
}
