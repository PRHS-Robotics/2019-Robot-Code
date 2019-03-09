/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/Elevator.h"
#include "Robot.h"
#include <iostream>
#include "subsystems/ElevatorDriveTrain.h"
#include "subsystems/DriveTrain.h"
Elevator::Elevator(Input *input) :
  m_input(input),
  // Use Requires() here to declare subsystem dependencies
  // eg. Requires(Robot::chassis.get());
  Command("Elevator", *static_cast< frc::Subsystem* >(Robot::m_driveTrain.get()))
{

}

// Called just before this Command runs the first time
void Elevator::Initialize() {
  
}

// Called repeatedly when this Command is scheduled to run
void Elevator::Execute() {
  //frc::Talon el_motorup{0};
  //el_motorup.Set(0.1);
  //el_motorup.Check();
  Robot::m_elevatordrivetrain->drive();
}

// Make this return true when this Command no longer needs to run execute()
bool Elevator::IsFinished() { 
  return false; }

// Called once after isFinished returns true
void Elevator::End() {

}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void Elevator::Interrupted() {
  std::cout << "END";
  End();
}
