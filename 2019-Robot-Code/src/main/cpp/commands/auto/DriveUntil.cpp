/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/auto/DriveUntil.h"
#include "Robot.h"

const double maxOutput = 0.2;

DriveUntil::DriveUntil(double targetDistance) :
  /*m_pidOutput(),
  m_pidInput(),
  m_controller(0.05, 0.0, 0.0, m_pidInput, m_pidOutput),*/
  m_targetDistance(targetDistance),
  frc::Command("DriveUntil", *Robot::m_driveTrain)
{
  /*m_controller.SetPIDSourceType(frc::PIDSourceType::kDisplacement);
  m_controller.SetSetpoint(targetDistance);
  m_controller.SetOutputRange(-maxOutput, maxOutput);
  m_controller.SetAbsoluteTolerance(1.0);*/
}

// Called just before this Command runs the first time
void DriveUntil::Initialize() {
  /*m_controller.SetEnabled(true);*/
  Robot::m_driveTrain->drive(0.0, 0.0);
}

// Called repeatedly when this Command is scheduled to run
void DriveUntil::Execute() {
  if (Robot::m_ultrasonic->GetVoltage() / (5.0 / 512.0) - m_targetDistance >= 21.0) {
    Robot::m_driveTrain->drive(0.4, 0.4);
  }
  else {
    Robot::m_driveTrain->drive(0.1, 0.1);
  }
}

// Make this return true when this Command no longer needs to run execute()
bool DriveUntil::IsFinished() {
  //return m_controller.OnTarget();
  return (Robot::m_ultrasonic->GetAverageVoltage() / (5.0 / 512.0)) - m_targetDistance <= 5.0;
}

// Called once after isFinished returns true
void DriveUntil::End() {
  Robot::m_driveTrain->drive(0.0, 0.0);
  /*m_controller.SetEnabled(false);
  Robot::m_driveTrain->drive(0.0, 0.0);*/
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void DriveUntil::Interrupted() {
  End();
}

/*void DriveUntil::DriveTrainOutput::PIDWrite(double output) {
  Robot::m_driveTrain->drive(-output, -output);
}

double DriveUntil::UltrasonicInput::PIDGet() {
  double distance = Robot::m_ultrasonic->GetVoltage() / (5.0 / 512.0);
  if (GetPIDSourceType() == frc::PIDSourceType::kDisplacement) {
    m_lastDistance = distance;
    return distance;
  }
  else {
    // Returns change in distance and updates previous distance
    return distance - std::exchange(m_lastDistance, distance);
  }
}

DriveUntil::UltrasonicInput::UltrasonicInput() :
  m_lastDistance(Robot::m_ultrasonic->GetVoltage() / (5.0 / 512.0)),
  frc::PIDSource()
{
  
}*/