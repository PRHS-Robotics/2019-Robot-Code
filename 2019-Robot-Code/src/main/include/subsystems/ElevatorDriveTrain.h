/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
#ifndef SRC_SUBSYSTEMS_ELEVATORDRIVETRAIN_H_
#define SRC_SUBSYSTEMS_ELEVATORDRIVETRAIN_H_


#include "subsystems/Input.h"
#include <frc/commands/Subsystem.h>
#include <ctre/Phoenix.h>
#include <frc/Talon.h>
class ElevatorDriveTrain : public frc::Subsystem {
 public:
  ElevatorDriveTrain(int motorup, int motordrive);
  void drive();
  
  //frc::PWMSpeedController motor1();
private:
  frc::Talon el_motorup{0};
  frc::Talon el_motordrive;
};
#endif /*SRC_SUBSYSTEMS_ELEVATORDRIVETRAIN_H_*/
