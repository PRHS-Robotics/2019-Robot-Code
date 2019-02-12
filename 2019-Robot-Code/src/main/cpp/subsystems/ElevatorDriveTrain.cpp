#include "Robot.h"
#include "subsystems/ElevatorDriveTrain.h"
//#include <ctre/Phoenix.h>
#include "subsystems/Drivetrain.h"
#include <frc/Talon.h>
#include <frc/PWMSpeedController.h>
//#include <SmartDashboard/SmartDashboard.h>
#include <iostream>

ElevatorDriveTrain::ElevatorDriveTrain(int motorup, int motordrive):
el_motordrive(motordrive),
el_motorup(motorup),
Subsystem("ElevatorDriveTrain")
{
  
  //frc::Scheduler::GetInstance()->RegisterSubsystem(this);
}


void ElevatorDriveTrain::drive() {
  //std::cout << "ELEVATING"
  
  //el_motordrive.SetSpeed(0.1);
  //Robot::m_driveTrain->drive(0.1, 0.1, false);
}
