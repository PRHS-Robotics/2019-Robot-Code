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
#include <SmartDashboard/SmartDashboard.h>

const double maxTurnPower = 0.5;

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
void TurnToAngle::Initialize() {
  m_error = 0.0;
}

// Called repeatedly when this Command is scheduled to run
void TurnToAngle::Execute() {
  double angle_difference = minDifference(Robot::getHeading(), m_target);

  std::cout << "angle diff: " << angle_difference << "\n";

  m_error += angle_difference / 15.0;

  std::cout << "error: " << m_error << "\n";

  frc::SmartDashboard::PutNumber("Angle Error", m_error);
  
  double turn = 1.0 * (-1.0/20.0) * (angle_difference + m_error);

  frc::SmartDashboard::PutNumber("Turn", turn);

  turn = std::max(-maxTurnPower, std::min(maxTurnPower, turn));

  Robot::m_driveTrain->drive(turn, -turn);
}

// Make this return true when this Command no longer needs to run execute()
bool TurnToAngle::IsFinished() {
  bool newResult = std::abs(minDifference(Robot::getHeading(), m_target)) <= 1.0;
  std::cout << "heading: " << Robot::getHeading() << ", " << minDifference(Robot::getHeading(), m_target) << "\n";
  static bool lastResult = false;
  return std::exchange(lastResult, newResult) && newResult;
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
