#include "Robot.h"
#include "subsystems/ElevatorDriveTrain.h"
//#include <ctre/Phoenix.h>
#include "subsystems/DriveTrain.h"
#include <frc/commands/Scheduler.h>
//#include <SmartDashboard/SmartDashboard.h>
#include <iostream>

ElevatorDriveTrain::ElevatorDriveTrain(int extender, int driveMotor) :
  m_extender(extender),
  m_driveMotor(driveMotor),
  Subsystem("ElevatorDriveTrain")
{
  
  frc::Scheduler::GetInstance()->RegisterSubsystem(this);
}


void ElevatorDriveTrain::drive() {
  //std::cout << "ELEVATING"
  
  //el_motordrive.SetSpeed(0.1);
  //Robot::m_driveTrain->drive(0.1, 0.1, false);
}