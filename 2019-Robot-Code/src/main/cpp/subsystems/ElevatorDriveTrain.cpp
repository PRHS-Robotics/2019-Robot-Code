#include "Robot.h"
#include "subsystems/ElevatorDriveTrain.h"
//#include <ctre/Phoenix.h>
#include "subsystems/DriveTrain.h"
#include <frc/commands/Scheduler.h>
//#include <SmartDashboard/SmartDashboard.h>
#include <iostream>

ElevatorDriveTrain::ElevatorDriveTrain(int extender, int driveMotor, int extendSolenoid, int retractSolenoid, int heightSensor) :
  m_extender(extender),
  m_driveMotor(driveMotor),
  m_extendSolenoid(extendSolenoid),
  m_retractSolenoid(retractSolenoid),
  m_heightSensor(heightSensor),
  Subsystem("ElevatorDriveTrain")
{
  m_heightSensor.SetAverageBits(12);

  m_extender.Set(0.0);
  m_driveMotor.Set(0.0);
  m_retractSolenoid.Set(true);
  m_extendSolenoid.Set(false);

  frc::Scheduler::GetInstance()->RegisterSubsystem(this);
}

void ElevatorDriveTrain::drive(double speed) {
  m_driveMotor.Set(speed);
}

void ElevatorDriveTrain::extend(double speed) {
  m_extender.Set(speed);
}

void ElevatorDriveTrain::setSolenoid(bool state) {
  m_extendSolenoid.Set(state);
  m_retractSolenoid.Set(!state);
}

double ElevatorDriveTrain::getHeight() {
  return m_heightSensor.GetAverageVoltage();
}